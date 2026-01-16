#!/usr/bin/env python3
import json, os, re, math
from typing import Any, Dict, List, Optional, Tuple
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

from .auctioneer import Auction, load_tasks


def robot_num(rid: str) -> Optional[int]:
    m = re.search(r"(\d+)$", str(rid))
    return int(m.group(1)) if m else None


def kind_str(k: Any) -> str:
    return (k.name if hasattr(k, "name") else str(k)).split(".")[-1].upper()


class SimpleExecutor(Node):
    def __init__(self, robot_id: str, schedule: Dict[str, Any]):
        super().__init__(f"executor_{robot_id}")

        # params (kept minimal)
        for k, v in {
            "arrival_tol": 0.15,
            "k_p": 0.8,
            "max_vel": 0.25,
            "rate_hz": 20.0,
            "align_to_now": True,
            "sync_mr_start": True,
        }.items():
            self.declare_parameter(k, v)

        gp = lambda n: self.get_parameter(n).value
        self.arrival_tol = float(gp("arrival_tol"))
        self.k_p = float(gp("k_p"))
        self.max_vel = float(gp("max_vel"))
        self.rate_hz = float(gp("rate_hz"))
        self.align_to_now = bool(gp("align_to_now"))
        self.sync_mr_start = bool(gp("sync_mr_start"))

        self.robot_id = robot_id
        self.tasks, _ = load_tasks("/root/ros2_ws/src/mrs_auction/config/tasks.yaml")

        self.pose: Optional[PoseStamped] = None
        self.pub = None
        
        now = self._now()
        self.t0 = now if self.align_to_now else 0.0

        # Build timeline for this robot only
        n = robot_num(robot_id)
        if n is None:
            raise ValueError(f"Invalid robot id: {robot_id}")
        
        ns = f"cf_{n}"
        self.pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.create_subscription(PoseStamped, f"/{ns}/pose", self._pose_cb, 10)

        # Parse schedule for this robot
        info = schedule.get(robot_id, {"name": [], "time": []})
        self.timeline = []
        for task_id, (s, f) in zip(info["name"], info["time"]):
            spec = self.tasks[task_id]
            self.timeline.append(
                dict(
                    task_id=task_id,
                    start=float(s),
                    dur=max(0.0, float(f) - float(s)),
                    loc=[float(spec.location[0]), float(spec.location[1])],
                    mr=(kind_str(spec.kind) == "MR") or (int(spec.robots_required) > 1),
                )
            )
        self.timeline.sort(key=lambda x: x["start"])
        
        self.idx = 0
        self.started = None
        self.enroute = None

        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.tick)
        self.get_logger().info(f"Executor ready for {robot_id}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _pose_cb(self, msg: PoseStamped):
        if self.pose is None:
            self.get_logger().info(f"{self.robot_id}: pose received")
        self.pose = msg

    def hover(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0 
        self.pub.publish(msg)

    def dist(self, g: List[float]) -> float:
        if self.pose is None:
            return 1e9
        dx = g[0] - self.pose.pose.position.x
        dy = g[1] - self.pose.pose.position.y
        return math.hypot(dx, dy)

    def drive(self, g: List[float]):
        if self.pose is None:
            self.hover()
            return

        ex = g[0] - self.pose.pose.position.x
        ey = g[1] - self.pose.pose.position.y
        vx, vy = self.k_p * ex, self.k_p * ey

        sp = math.hypot(vx, vy)
        if sp > self.max_vel:
            k = self.max_vel / max(sp, 1e-6)
            vx, vy = vx * k, vy * k

        msg = Twist()
        msg.linear.x, msg.linear.y = float(vx), float(vy)
        msg.linear.z = 0.0
        self.pub.publish(msg)

    def tick(self):
        now = self._now()

        if self.idx >= len(self.timeline):
            self.hover()
            return

        it = self.timeline[self.idx]
        task, g = it["task_id"], it["loc"]
        start = it["start"] + self.t0
        dur = it["dur"]

        arrived = self.dist(g) <= self.arrival_tol

        # travel if not arrived
        if not arrived:
            if self.enroute != task:
                self.get_logger().info(f"{self.robot_id}: heading {task} ({g[0]:.1f},{g[1]:.1f})")
                self.enroute = task
            self.drive(g)
            return

        # arrived early -> wait until start
        if now < start:
            self.hover()
            return

        # start service once
        if self.started is None:
            self.started = max(now, start)
            self.enroute = None
            self.get_logger().info(f"{self.robot_id}: service start {task}")
            self.hover()
            return

        # finish
        if now >= self.started + dur:
            self.get_logger().info(f"{self.robot_id}: service done {task}")
            self.started = None
            self.idx += 1
            self.hover()
        else:
            self.hover()


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    if len(sys.argv) < 2:
        print("Usage: ros2 run mrs_auction simple_executor_node <robot_id>")
        return
    
    robot_id = sys.argv[1]
    
    # Load existing schedule
    schedule_dir = "/root/ros2_ws/src/mrs_auction/schedule"
    schedule_files = sorted([f for f in os.listdir(schedule_dir) if f.startswith("schedule_") and f.endswith(".json")])
    
    if not schedule_files:
        print("No schedule found! Run auction first.")
        return
    
    latest_schedule = os.path.join(schedule_dir, schedule_files[-1])
    with open(latest_schedule, "r") as f:
        sol = json.load(f)
    
    if robot_id not in sol:
        print(f"Robot {robot_id} not in schedule!")
        return
    
    node = SimpleExecutor(robot_id, sol)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
