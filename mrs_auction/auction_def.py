#!/usr/bin/env python3
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple
import math
import networkx as nx


def dist(p1: Sequence[float], p2: Sequence[float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


@dataclass(frozen=True)
class TaskSpec:
    task_id: str
    location: Tuple[float, float]
    duration_s: float
    kind: str = "SR"                 # "SR" or "MR"
    robots_required: int = 1
    formation: Optional[str] = None
    deps: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    start_location: Tuple[float, float]
    speed_m_s: float


@dataclass
class TaskEntry:
    task_id: str
    service_duration: float

    # For locking / constraints
    EST: float = 0.0
    LFT: float = float("inf")  # Latest finish allowed
    fixed: bool = False

    # Scheduled times
    start: float = 0.0
    finish: float = 0.0


class PrecedenceGraph(nx.DiGraph):
    """
    Stores precedence constraints. update_layers() gives:
    Tf: free tasks (no remaining predecessors),
    Tl: next layer (all predecessors are in Tf),
    Th: remaining.
    """

    def __init__(self, tasks: Dict[str, TaskSpec], relations: List[Tuple[str, str]]):
        super().__init__()
        for tid, spec in tasks.items():
            self.add_node(tid, info=spec)
        self.add_edges_from(relations)

        self.Tf: List[str] = []
        self.Tl: List[str] = []
        self.Th: List[str] = []

    def update_layers(self) -> None:
        Tf = {n for n in self.nodes if self.in_degree(n) == 0}

        Tl = set()
        for n in self.nodes:
            if n in Tf:
                continue
            preds = set(self.predecessors(n))
            if preds and preds.issubset(Tf):
                Tl.add(n)

        Th = set(self.nodes) - Tf - Tl
        self.Tf = sorted(Tf)
        self.Tl = sorted(Tl)
        self.Th = sorted(Th)


class Schedule:
    def __init__(self) -> None:
        self.items: List[TaskEntry] = []

    def __len__(self) -> int:
        return len(self.items)

    def __repr__(self) -> str:
        return repr(self.items)

    @property
    def makespan(self) -> float:
        return self.items[-1].finish if self.items else 0.0

    def locked_prefix_len(self) -> int:
        """Count fixed tasks from the start (previous layers)."""
        k = 0
        for e in self.items:
            if e.fixed:
                k += 1
            else:
                break
        return k

    def recompute_from(
        self,
        start_idx: int,
        robot: RobotSpec,
        tasks: Dict[str, TaskSpec],
        pred_finish: Dict[str, float],
    ) -> None:
        """
        Correct travel handling:
        arrival = prev.finish + travel_time
        start  = max(arrival, pred_finish[task], EST)
        finish = start + service_duration
        """
        if not self.items:
            return

        start_idx = max(0, min(start_idx, len(self.items) - 1))

        for j in range(start_idx, len(self.items)):
            tid = self.items[j].task_id
            loc = tasks[tid].location

            if j == 0:
                prev_loc = robot.start_location
                prev_finish = 0.0
            else:
                prev_tid = self.items[j - 1].task_id
                prev_loc = tasks[prev_tid].location
                prev_finish = self.items[j - 1].finish

            if robot.speed_m_s <= 0:
                travel = float("inf")
            else:
                travel = dist(prev_loc, loc) / robot.speed_m_s

            arrival = prev_finish + travel
            start = max(self.items[j].EST, pred_finish.get(tid, 0.0), arrival)

            self.items[j].start = start
            self.items[j].finish = start + self.items[j].service_duration

    def insert_task(
        self,
        task_id: str,
        index: int,
        robot: RobotSpec,
        tasks: Dict[str, TaskSpec],
        pred_finish: Dict[str, float],
    ) -> None:
        entry = TaskEntry(task_id=task_id, service_duration=tasks[task_id].duration_s)
        self.items.insert(index, entry)
        self.recompute_from(index, robot, tasks, pred_finish)

    def set_task_start(
        self,
        task_id: str,
        new_start: float,
        robot: RobotSpec,
        tasks: Dict[str, TaskSpec],
        pred_finish: Dict[str, float],
    ) -> None:
        
        idx = next((i for i, e in enumerate(self.items) if e.task_id == task_id), None)
        if idx is None:
            return

        # compute arrival constraint again
        if idx == 0:
            prev_loc = robot.start_location
            prev_finish = 0.0
        else:
            prev_tid = self.items[idx - 1].task_id
            prev_loc = tasks[prev_tid].location
            prev_finish = self.items[idx - 1].finish

        if robot.speed_m_s <= 0:
            travel = float("inf")
        else:
            travel = dist(prev_loc, tasks[task_id].location) / robot.speed_m_s

        arrival = prev_finish + travel
        start = max(new_start, self.items[idx].EST, pred_finish.get(task_id, 0.0), arrival)

        self.items[idx].start = start
        self.items[idx].finish = start + self.items[idx].service_duration

        if idx + 1 < len(self.items):
            self.recompute_from(idx + 1, robot, tasks, pred_finish)

    def is_valid(self) -> bool:
        eps = 1e-9
        for e in self.items:
            if e.finish > e.LFT + eps:
                return False
        return True

    def tighten(self) -> Dict[str, float]:
        """Lock current-layer tasks by setting LFT=finish and fixed=True."""
        fixed_times: Dict[str, float] = {}
        for e in self.items:
            if not e.fixed:
                e.fixed = True
                e.LFT = e.finish
                fixed_times[e.task_id] = e.finish
        return fixed_times
