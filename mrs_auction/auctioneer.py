#!/usr/bin/env python3
import itertools
import json
import os
import yaml
from datetime import datetime
from typing import Any, Dict, List, Tuple

from mrs_auction.auction_def import PrecedenceGraph, RobotSpec, TaskSpec
from mrs_auction.bidder import Bidder, BidResult


class Auctioneer:
    def __init__(self, precedence_graph: PrecedenceGraph, tasks: Dict[str, TaskSpec], bidders: Dict[str, Bidder]):
        self.G = precedence_graph
        self.tasks = tasks
        self.bidders = bidders
        self.task_assignments: Dict[str, List[str]] = {}

    def _best_allocation_for_task(
        self,
        task_id: str,
        pred_finish: Dict[str, float],
    ) -> Tuple[float, float, List[BidResult], float] | None:
        """
        Returns:
          (objective, tie, chosen_bids, sync_start)

        objective:
          SR -> winner makespan
          MR -> max(adjusted makespan) after syncing to latest start

        adjusted makespan = bid.makespan + wait
        wait = max(0, sync_start - bid.task_start)
        """
        required = max(1, int(self.tasks[task_id].robots_required))

        # Collect bids
        all_bids: List[BidResult] = []
        for bidder in self.bidders.values():
            br = bidder.evaluate_task(task_id, pred_finish)
            if br is not None:
                all_bids.append(br)

        if len(all_bids) < required:
            return None

        best: Tuple[float, float, List[BidResult], float] | None = None

        # exact combination search
        for combo in itertools.combinations(all_bids, required):
            sync_start = max(b.task_start for b in combo)
            adjusted = [b.makespan + max(0.0, sync_start - b.task_start) for b in combo]
            objective = max(adjusted)
            tie = sum(adjusted)

            cand = (objective, tie, list(combo), sync_start)
            if best is None or cand[:2] < best[:2]:
                best = cand

        return best

    def start_auction(self) -> None:
        scheduled: set[str] = set()
        pred_finish: Dict[str, float] = {tid: 0.0 for tid in self.tasks}

        while len(scheduled) < len(self.tasks):
            self.G.update_layers()
            Tf = [t for t in self.G.Tf if t not in scheduled]

            if not Tf:
                raise RuntimeError("No free tasks available (cycle or missing deps).")

            # Allocate all tasks in this free layer (TF)
            scheduled_new: List[str] = []
            remaining = Tf[:]

            while remaining:
                best_overall: Tuple[float, float, str, List[BidResult], float] | None = None
                for task_id in remaining:
                    res = self._best_allocation_for_task(task_id, pred_finish)
                    if res is None:
                        continue
                    objective, tie, chosen_bids, sync_start = res
                    cand = (objective, tie, task_id, chosen_bids, sync_start)
                    if best_overall is None or cand[:2] < best_overall[:2]:
                        best_overall = cand

                if best_overall is None:
                    raise RuntimeError(f"No feasible allocation in TF: {remaining}")

                _, _, task_id, chosen_bids, sync_start = best_overall
                winner_ids = [b.robot_id for b in chosen_bids]

                # Commit schedules for winners
                for b in chosen_bids:
                    self.bidders[b.robot_id].commit(b)

                # MR sync: push everyone to same start
                if len(chosen_bids) > 1:
                    for b in chosen_bids:
                        self.bidders[b.robot_id].sync_task_start(task_id, sync_start, pred_finish)

                self.task_assignments[task_id] = winner_ids
                scheduled_new.append(task_id)
                remaining.remove(task_id)

            # Lock schedules after TF is fully assigned
            layer_finish_times: Dict[str, float] = {}
            for bidder in self.bidders.values():
                layer_finish_times.update(bidder.lock_layer())

            # Propagate precedence and shrink graph
            for t in scheduled_new:
                scheduled.add(t)
                ft = layer_finish_times.get(t, 0.0)
                for succ in list(self.G.successors(t)):
                    pred_finish[succ] = max(pred_finish[succ], ft)
                if t in self.G:
                    self.G.remove_node(t)


class Auction:
    def __init__(self, tasks_yaml: str, robots_yaml: str):
        tasks, relations = load_tasks(tasks_yaml)
        robots = load_robots(robots_yaml)

        self.tasks = tasks
        self.robots = robots

        G = PrecedenceGraph(tasks, relations)
        bidders = {r.robot_id: Bidder(r, tasks) for r in robots}
        self.auctioneer = Auctioneer(G, tasks, bidders)

    def make_solution(self) -> Dict[str, Dict[str, List[Any]]]:
        self.auctioneer.start_auction()

        solution: Dict[str, Dict[str, List[Any]]] = {}
        for r in self.robots:
            sch = self.auctioneer.bidders[r.robot_id].fixed_schedule
            solution[r.robot_id] = {"name": [], "time": []}
            for e in sch.items:
                solution[r.robot_id]["name"].append(e.task_id)
                solution[r.robot_id]["time"].append((e.start, e.finish))

        return solution


def load_tasks(path: str) -> Tuple[Dict[str, TaskSpec], List[Tuple[str, str]]]:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    tasks: Dict[str, TaskSpec] = {}
    relations: List[Tuple[str, str]] = []

    for t in data.get("tasks", []):
        tid = t.get("id")
        if not tid:
            continue

        loc = t.get("location", [0.0, 0.0])
        location = (float(loc[0]), float(loc[1]))

        kind = str(t.get("type", "SR"))
        robots_required = int(t.get("numb_robots", 1))
        duration_s = float(t.get("duration_s", 0.0))
        formation = t.get("formation", None)
        deps = [str(d) for d in t.get("deps", [])]

        tasks[tid] = TaskSpec(
            task_id=tid,
            location=location,
            duration_s=duration_s,
            kind=kind,
            robots_required=robots_required,
            formation=formation,
            deps=deps,
        )

        for dep in deps:
            relations.append((dep, tid))

    return tasks, relations


def load_robots(path: str) -> List[RobotSpec]:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    robots: List[RobotSpec] = []
    for r in data.get("robots", []):
        rid = str(r.get("id"))
        loc = r.get("start_location", [0.0, 0.0])
        start_location = (float(loc[0]), float(loc[1]))
        speed = float(r.get("speed_m_s", 0.0))
        robots.append(RobotSpec(robot_id=rid, start_location=start_location, speed_m_s=speed))

    return robots


if __name__ == "__main__":
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    auction = Auction(
        tasks_yaml=os.path.join(config_dir, 'tasks.yaml'),
        robots_yaml=os.path.join(config_dir, 'robots.yaml'),
    )
    sol = auction.make_solution()
    print("Final solution:", sol)
    
    # Save the solution to a JSON file
    schedule_dir = os.path.join(os.path.dirname(__file__), '..', 'schedule')
    os.makedirs(schedule_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    schedule_file = os.path.join(schedule_dir, f"schedule_{timestamp}.json")
    with open(schedule_file, 'w') as f:
        json.dump(sol, f, indent=4)
    print(f"Schedule saved to {schedule_file}")
