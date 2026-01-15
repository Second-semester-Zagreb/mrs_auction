#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Dict, List, Optional
from copy import deepcopy

from .auction_def import RobotSpec, Schedule, TaskSpec


@dataclass(frozen=True)
class BidResult:
    robot_id: str
    task_id: str
    makespan: float
    task_start: float
    schedule: Schedule


class Bidder:
    def __init__(self, robot: RobotSpec, tasks: Dict[str, TaskSpec]) -> None:
        self.robot = robot
        self.tasks = tasks
        self.fixed_schedule = Schedule()

    def evaluate_task(self, task_id: str, pred_finish: Dict[str, float]) -> Optional[BidResult]:
        """
        Compute the best insertion of task_id into the current schedule
        """
        base = deepcopy(self.fixed_schedule)
        locked = base.locked_prefix_len()

        best: Optional[BidResult] = None

        for idx in range(locked, len(base.items) + 1):
            temp = deepcopy(base)
            temp.insert_task(task_id, idx, self.robot, self.tasks, pred_finish)

            if not temp.is_valid():
                continue

            tstart = next(e.start for e in temp.items if e.task_id == task_id)
            cand = BidResult(
                robot_id=self.robot.robot_id,
                task_id=task_id,
                makespan=temp.makespan,
                task_start=tstart,
                schedule=temp,
            )

            if best is None or cand.makespan < best.makespan:
                best = cand

        return best

    def commit(self, bid: BidResult) -> None:
        """Commit the schedule returned by evaluate_task()."""
        self.fixed_schedule = deepcopy(bid.schedule)

    def sync_task_start(self, task_id: str, sync_start: float, pred_finish: Dict[str, float]) -> None:
        """MR: force the task to start at sync_start, then recompute downstream."""
        self.fixed_schedule.set_task_start(task_id, sync_start, self.robot, self.tasks, pred_finish)

    def lock_layer(self) -> Dict[str, float]:
        """Called once after TF is fully assigned."""
        return self.fixed_schedule.tighten()
