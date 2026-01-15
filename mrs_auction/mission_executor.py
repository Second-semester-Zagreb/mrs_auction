#!/usr/bin/env python3

import json
import os
import re
import yaml
from datetime import datetime
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from std_msgs.msg import Int32MultiArray
from crazyflie_interfaces.srv import Land, GoTo, NotifySetpointsStop
from builtin_interfaces.msg import Duration

from .auctioneer import Auction, load_tasks


def robot_num(robot_id: str) -> Optional[int]:
    m = re.search(r"(\d+)$", robot_id)
    return int(m.group(1)) if m else None


def get_formation_for_robot_count(num_robots: int) -> str:
    return {1: "rendezvous", 2: "horizontal_line", 3: "circle", 4: "square"}.get(num_robots, "circle")


class MissionExecutor(Node):
    def __init__(self, solution: Dict[str, Any]):
        super().__init__("mission_executor")

        self.declare_parameter("arrival_tol", 0.15)
        self.declare_parameter("k_p", 0.8)
        self.declare_parameter("max_vel", 0.2)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("align_to_now", True)

        # Consensus leader id 
        self.declare_parameter("sr_robot_id", 2)

        # Landing
        self.declare_parameter("land_hold", 2.0)
        self.land_hold = float(self.get_parameter("land_hold").value)

        self.go_to_cli: Dict[str, Any] = {}
        self.land_cli: Dict[str, Any] = {}
        self.notify_stop_cli: Dict[str, Any] = {}

        self.declare_parameter("return_z", 1.0)
        self.declare_parameter("return_time_s", 5.0)
        self.declare_parameter("land_height", 0.05)
        self.declare_parameter("land_time_s", 3.0)

        self.return_z = float(self.get_parameter("return_z").value)
        self.return_time_s = float(self.get_parameter("return_time_s").value)
        self.land_height = float(self.get_parameter("land_height").value)
        self.land_time_s = float(self.get_parameter("land_time_s").value)



        self.arrival_tol = float(self.get_parameter("arrival_tol").value)
        self.k_p = float(self.get_parameter("k_p").value)
        self.max_vel = float(self.get_parameter("max_vel").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.align_to_now = bool(self.get_parameter("align_to_now").value)
        self.sr_robot_id = int(self.get_parameter("sr_robot_id").value)
        
        self.schedule: Dict[str, Dict[str, Any]] = solution or {}

        # Load tasks
        tasks_path = "/root/ros2_ws/src/mrs_auction/config/tasks.yaml"
        tasks_map, _ = load_tasks(tasks_path)  # Dict[str, TaskSpec]
        self.tasks = tasks_map

        # Load robots' specs
        robots_path = "/root/ros2_ws/src/mrs_auction/config/robots.yaml"
        with open(robots_path, 'r') as f:
            robots_config = yaml.safe_load(f)
        self.home_locations: Dict[str, List[float]] = {}
        for r in robots_config.get("robots", []):
            rid = str(r.get("id"))
            loc = r.get("start_location", [0.0, 0.0])
            self.home_locations[rid] = [float(loc[0]), float(loc[1])]

        # Robot IO
        self.pose: Dict[str, Optional[PoseStamped]] = {}
        self.cmd_pub: Dict[str, Any] = {}

        # Consensus control topics
        self.virtual_enable_pub = self.create_publisher(Bool, "/virtual_robot/enable", 10)
        self.virtual_leader_pub = self.create_publisher(Bool, "/virtual_robot/leader", 10)
        self.virtual_goal_pub = self.create_publisher(PoseStamped, "/virtual_robot/goal", 10)

        # Just activate the robots to do the formation in consensus code
        self.active_robots_pub = self.create_publisher(Int32MultiArray, '/consensus/active_robots', 10)
        self.last_active_set: Set[int] = set()
        # Formation type topic (we added subscriber in consensus.py)
        self.formation_type_pub = self.create_publisher(String, "/formation/type_request", 10)

        # Execution state
        self.robot_state: Dict[str, Dict[str, Any]] = {}
        self.task_to_robots: Dict[str, List[str]] = {}

        # MR tracking
        self.mr_started_at: Dict[str, Optional[float]] = {}
        self.mr_completed: Dict[str, Set[str]] = {}
        self.consensus_active_task: Optional[str] = None

        # Time alignment
        self.base_time = self.get_clock().now().nanoseconds / 1e9
        if self.schedule:
            min_start = min(float(s) for info in self.schedule.values() for (s, _f) in info["time"])
        else:
            min_start = 0.0
        self.time_offset = (self.base_time - min_start) if self.align_to_now else 0.0

        # Setup per robot
        for rid, info in self.schedule.items():
            n = robot_num(rid)
            if n is None:
                self.get_logger().warn(f"Robot id {rid} does not end with a number; skipping")
                continue

            ns = f"cf_{n}"
            self.cmd_pub[rid] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
            self.pose[rid] = None
            self.create_subscription(PoseStamped, f"/{ns}/pose", lambda msg, rid=rid: self._pose_cb(rid, msg), 10)
            self.go_to_cli[rid] = self.create_client(GoTo, f"/{ns}/go_to")
            self.land_cli[rid] = self.create_client(Land, f"/{ns}/land")
            self.notify_stop_cli[rid] = self.create_client(NotifySetpointsStop, f"/{ns}/notify_setpoints_stop")
            names = info["name"]
            times = info["time"]

            timeline = []
            for task_id, (s, f) in zip(names, times):
                spec = self.tasks[task_id]
                planned_start = float(s)
                planned_finish = float(f)
                service_duration = max(0.0, planned_finish - planned_start)

                timeline.append(
                    {
                        "task_id": task_id,
                        "planned_start": planned_start,
                        "service_duration": service_duration,
                        "location": [float(spec.location[0]), float(spec.location[1])],
                        "type": str(spec.kind),
                        "numb_robots": int(spec.robots_required),
                        "formation": spec.formation,
                    }
                )

            timeline.sort(key=lambda x: x["planned_start"])

            self.robot_state[rid] = {
                "timeline": timeline,
                "index": 0,
                "sr_started_at": None,  # SR actual start time
                "ns": ns,
                "end_mode": "WORK", #  WORK | RETURN | LAND | DONE
                "return_sent": False,
                "return_sent_at": None,
                "land_sent": False,
                "land_sent_at": None,
            }

            for item in timeline:
                self.task_to_robots.setdefault(item["task_id"], []).append(rid)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self.tick)
        self.get_logger().info("Mission executor ready")

    def _pose_cb(self, rid: str, msg: PoseStamped):
        self.pose[rid] = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def publish_virtual(self, enable: bool, leader: bool, goal_xy: Optional[List[float]]):
        en = Bool()
        en.data = bool(enable)
        self.virtual_enable_pub.publish(en)

        ld = Bool()
        ld.data = bool(leader)
        self.virtual_leader_pub.publish(ld)

        if goal_xy is not None:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(goal_xy[0])
            msg.pose.position.y = float(goal_xy[1])
            msg.pose.position.z = 1.0
            self.virtual_goal_pub.publish(msg)

    def publish_formation(self, formation: Optional[str], num_robots: int):
        if not formation:
            formation = get_formation_for_robot_count(num_robots)
        fm = String()
        fm.data = formation
        self.formation_type_pub.publish(fm)

    def publish_active_robots(self, robot_ids: List[str]):
        msg = Int32MultiArray()
        nums = []
        for rid in robot_ids:
            n = robot_num(rid)
            if n is not None:
                nums.append(n)

        new_set = set(nums)
        if new_set == self.last_active_set:
            return
        self.last_active_set = new_set
        
        msg.data = sorted(nums)
        self.active_robots_pub.publish(msg)

    def all_sr_done(self) -> bool:
        for rid, state in self.robot_state.items():
            idx = state["index"]
            tl = state["timeline"]
            if idx < len(tl):
                item = tl[idx]
                is_mr = (item["type"] == "MR") or (int(item["numb_robots"]) > 1)
                if not is_mr:
                    return False
        return True

    def stop(self, rid: str):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.cmd_pub[rid].publish(msg)

    def drive_towards(self, rid: str, loc_xy: List[float]):
        if self.pose.get(rid) is None:
            return
        px = self.pose[rid].pose.position.x
        py = self.pose[rid].pose.position.y
        ex = loc_xy[0] - px
        ey = loc_xy[1] - py

        vx = max(-self.max_vel, min(self.max_vel, self.k_p * ex))
        vy = max(-self.max_vel, min(self.max_vel, self.k_p * ey))

        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        self.cmd_pub[rid].publish(msg)

    def duration_msg(self, seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        d = Duration()
        d.sec = sec
        d.nanosec = nanosec
        return d

    def go_to_home(self, rid: str) -> bool:
        home = self.home_locations.get(rid)
        if home is None:
            self.get_logger().warn(f"{rid}: no home location known")
            return False
        client = self.go_to_cli.get(rid)
        if client is None:
            self.get_logger().warn(f"{rid}: go_to client missing")
            return False
        if not client.service_is_ready():
            self.get_logger().warn(f"{rid}: /{self.robot_state[rid]['ns']}/go_to not ready")
            return False

        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal.x = float(home[0])
        req.goal.y = float(home[1])
        req.goal.z = float(self.return_z)
        req.yaw = 0.0  # degrees
        req.duration = self.duration_msg(self.return_time_s)

        client.call_async(req)
        self.get_logger().info(f"{rid}: go_to home ({home[0]:.2f},{home[1]:.2f},{self.return_z:.2f})")
        return True
    
    def land(self, rid: str):
        client = self.land_cli.get(rid)
        if client is None:
            self.get_logger().warn(f"{rid}: land client missing")
            return None
        if not client.service_is_ready():
            self.get_logger().warn(f"{rid}: /{self.robot_state[rid]['ns']}/land not ready")
            return None

        req = Land.Request()
        req.group_mask = 0
        req.height = float(self.land_height)
        req.duration = self.duration_msg(self.land_time_s)

        future = client.call_async(req)
        self.get_logger().info(f"{rid}: land requested (duration={self.land_time_s:.1f}s)")
        return future

    def dist_to(self, rid: str, loc_xy: List[float]) -> float:
        if self.pose.get(rid) is None:
            return 1e9
        px = self.pose[rid].pose.position.x
        py = self.pose[rid].pose.position.y
        dx = loc_xy[0] - px
        dy = loc_xy[1] - py
        return (dx * dx + dy * dy) ** 0.5

    def tick(self):
        now = self._now()
        active_mr_task = None
        active_mr_goal = None
        active_mr_formation = None
        active_mr_n = 0

        if self.all_sr_done():
            for rid, state in self.robot_state.items():
                idx = state["index"]
                tl = state["timeline"]
                if idx >= len(tl):
                    continue
                item = tl[idx]
                is_mr = (item["type"] == "MR") or (int(item["numb_robots"]) > 1)
                if is_mr:
                    active_mr_task = item["task_id"]
                    active_mr_goal = item["location"]
                    active_mr_formation = item.get("formation")
                    active_mr_n = int(item["numb_robots"])
                    break

        # Manage consensus enable/disable globally
        if active_mr_task is not None:
            assigned_rids = self.task_to_robots.get(active_mr_task, [])
            # Only include robots that are currently on this MR task and not already finished/landing
            filtered = []
            for rid in assigned_rids:
                st = self.robot_state.get(rid)
                if not st:
                    continue
                idx = st.get("index", 0)
                tl = st.get("timeline", [])
                if idx >= len(tl):
                    continue  # already finished timeline
                cur = tl[idx]
                if cur.get("task_id") != active_mr_task:
                    continue
                if st.get("end_mode") != "WORK":
                    continue
                filtered.append(rid)

            if filtered:
                self.publish_active_robots(filtered)
                # Enable consensus and set goal + leader mode
                self.publish_virtual(enable=True, leader=True, goal_xy=active_mr_goal)
                self.publish_formation(active_mr_formation, active_mr_n)
                self.consensus_active_task = active_mr_task
            else:
                # Nobody left needing this MR task
                if self.consensus_active_task is not None:
                    self.publish_virtual(enable=False, leader=False, goal_xy=None)
                    self.publish_active_robots([])
                    self.consensus_active_task = None
        else:
            # No MR task active => disable consensus
            if self.consensus_active_task is not None:
                self.publish_virtual(enable=False, leader=False, goal_xy=None)
                self.publish_active_robots([])
                self.consensus_active_task = None

        # Execute per robot task
        for rid, state in self.robot_state.items():
            idx = state["index"]
            tl = state["timeline"]
            if idx >= len(tl):
                # Finished all tasks -> return home and land
                # Ensure this robot is not in active consensus control
                home = self.home_locations.get(rid)
                if home is None:
                    self.stop(rid)
                    continue

                mode = state.get("end_mode", "WORK")

                if mode == "WORK":
                    state["end_mode"] = "RETURN"
                    mode = "RETURN"
                    self.get_logger().info(f"{rid}: tasks finished -> RETURN home")

                if mode == "RETURN":
                    # send go_to once
                    if not state.get("return_sent", False):
                        if self.go_to_home(rid):
                            state["return_sent"] = True
                            state["return_sent_at"] = now
                    else:
                        # Fallback: if go_to service hasn't worked after 2 seconds, use drive_towards
                        if state.get("return_sent_at") is not None:
                            time_since_sent = now - state["return_sent_at"]
                            if time_since_sent > 2.0:
                                # go_to might have failed, use manual drive
                                self.drive_towards(rid, home)

                    # wait until near home
                    if self.dist_to(rid, home) > self.arrival_tol:
                        continue

                    # arrived home -> land; disable consensus influence
                    self.publish_virtual(enable=False, leader=False, goal_xy=None)
                    self.publish_active_robots([])
                    self.consensus_active_task = None
                    state["end_mode"] = "LAND"
                    mode = "LAND"
                    self.get_logger().info(f"{rid}: arrived home -> LAND")

                if mode == "LAND":
                    # Do not publish cmd_vel during landing

                    # Send notify_stop once to exit low-level control
                    if not state.get("land_sent", False):
                        notify_cli = self.notify_stop_cli.get(rid)
                        if notify_cli and notify_cli.service_is_ready():
                            req = NotifySetpointsStop.Request()
                            notify_cli.call_async(req)
                            state["land_sent_at"] = now
                            # small delay to let firmware switch modes
                            if (self._now() - state["land_sent_at"]) < 0.1:
                                continue

                    # Single land call (like test_land_takeoff)
                    if not state.get("land_sent", False):
                        future = self.land(rid)
                        print(f"LAND future: {future}")
                        if future:
                            state["land_sent"] = True
                            state["land_sent_at"] = now
                            self.get_logger().error(f"{rid}: call land service successful")
                        else:
                            self.get_logger().error(f"{rid}: failed to call land service")

                    # Wait land_time_s then mark DONE
                    if state.get("land_sent_at") is not None:
                        if (now - state["land_sent_at"]) >= self.land_time_s:
                            state["end_mode"] = "DONE"
                            self.get_logger().info(f"{rid}: landing complete -> DONE")
                    continue

                # DONE
                # self.stop(rid)
                continue


            item = tl[idx]
            task_id = item["task_id"]
            planned_start = float(item["planned_start"]) + self.time_offset
            duration = float(item["service_duration"])
            goal = item["location"]
            is_mr = (item["type"] == "MR") or (int(item["numb_robots"]) > 1)

            if not is_mr:
                # SR
                dist = self.dist_to(rid, goal)
                at_goal = dist <= self.arrival_tol

                if not at_goal:
                    self.drive_towards(rid, goal)
                    continue

                # At goal: wait for planned start
                if now < planned_start:
                    self.stop(rid)
                    continue

                # Start service if not started
                if state["sr_started_at"] is None:
                    state["sr_started_at"] = now
                    self.stop(rid)
                    self.get_logger().info(f"[SR] {rid} START {task_id}")
                    continue

                # Service running
                self.stop(rid)
                if now >= state["sr_started_at"] + duration:
                    self.get_logger().info(f"[SR] {rid} DONE {task_id}")
                    state["sr_started_at"] = None
                    state["index"] += 1

                continue

            # consensus controls motion
            assigned = set(self.task_to_robots.get(task_id, []))
            if not assigned:
                assigned = {rid}
            tol = max(self.arrival_tol, 2.0)

            # MR start time shared per task
            if task_id not in self.mr_started_at:
                self.mr_started_at[task_id] = None
                self.mr_completed[task_id] = set()

            if self.mr_started_at[task_id] is None:
                if now < planned_start:
                    continue

                all_close = True
                for r in assigned:
                    if r in self.robot_state:
                        if self.dist_to(r, goal) > tol:
                            all_close = False
                            break

                if all_close:
                    self.mr_started_at[task_id] = now
                    self.get_logger().info(f"[MR] START {task_id} robots={sorted(assigned)} t={now:.2f}")
                continue

            # Service running
            if now >= self.mr_started_at[task_id] + duration:
                if rid not in self.mr_completed[task_id]:
                    self.mr_completed[task_id].add(rid)
                    self.get_logger().info(f"[MR] {rid} DONE {task_id}")

                state["index"] += 1

                # If all assigned robots done -> finish MR task and disable consensus
                if assigned.issubset(self.mr_completed[task_id]):
                    self.get_logger().info(f"[MR] ALL DONE {task_id}, disabling consensus")
                    self.publish_virtual(enable=False, leader=False, goal_xy=None)
                    self.consensus_active_task = None
                    self.mr_started_at.pop(task_id, None)
                    self.mr_completed.pop(task_id, None)
                    self.publish_active_robots([])

        # ensure consensus is disabled when all robots are done
        if all(st.get("end_mode") == "DONE" for st in self.robot_state.values()):
            if self.consensus_active_task is not None:
                self.publish_virtual(enable=False, leader=False, goal_xy=None)
                self.publish_active_robots([])
                self.consensus_active_task = None



def main(args=None):
    rclpy.init(args=args)

    # # auction = Auction()
    # auction = Auction(
    #     tasks_yaml="/root/ros2_ws/src/mrs_auction/config/tasks.yaml",
    #     robots_yaml="/root/ros2_ws/src/mrs_auction/config/robots.yaml",
    # )
    # solution = auction.make_solution()

    # schedule_dir = "/root/ros2_ws/src/mrs_auction/schedule"
    # os.makedirs(schedule_dir, exist_ok=True)
    # timestamp = datetime.now().strftime("%H%M%d%m")
    # solution_file = os.path.join(schedule_dir, f"schedule_{timestamp}.json")
    # with open(solution_file, "w") as f:
    #     json.dump(solution, f, indent=2)
    # print(f"Solution saved to {solution_file}")
    # print("Final solution:", solution)
    # node = MissionExecutor(solution)
    # try:
    #     rclpy.spin(node)
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
