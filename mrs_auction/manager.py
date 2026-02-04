#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import json
import os
import yaml
from datetime import datetime
from geometry_msgs.msg import Point

class AuctionManager(Node):
    def __init__(self):
        super().__init__('auction_manager')
        
        # Parameters
        self.declare_parameter('schedule_file', '')
        self.declare_parameter('tasks_config', '')
        
        schedule_arg = self.get_parameter('schedule_file').get_parameter_value().string_value
        tasks_config_path = self.get_parameter('tasks_config').get_parameter_value().string_value
        
        # Resolve paths. Its a pain to get the paths right. 
        package_share = get_package_share_directory('mrs_auction')

        if not os.path.isabs(tasks_config_path):
            tasks_config_path = os.path.join(package_share, 'config', tasks_config_path if tasks_config_path else 'custom_tasks.yaml')
        
        if not os.path.exists(tasks_config_path):
            self.get_logger().error(f'Tasks config not found: {tasks_config_path}')
            return

        # Load tasks configuration to get locations and formations
        self.tasks_data = self.load_tasks_config(tasks_config_path)
        self.all_task_ids = list(self.tasks_data.keys())
        
        schedule_path = self.resolve_schedule_path(schedule_arg)
        if not schedule_path:
            return
        
        # Load schedule
        self.schedule = self.load_schedule(schedule_path)
        
        # flatten schedule into per-robot queues. Its how we discussed it. Each robot has a queue of tasks
        self.robot_queues = {rid: [] for rid in self.schedule.keys()}
        scheduled_robots_per_task = {} # tid -> count
        
        for rid, data in self.schedule.items():
            for i in range(len(data['name'])):
                tid = data['name'][i]
                self.robot_queues[rid].append({
                    'task_id': tid,
                    'start_time': data['time'][i][0],
                    'finish_time': data['time'][i][1]
                })
                if tid not in scheduled_robots_per_task:
                    scheduled_robots_per_task[tid] = 0
                scheduled_robots_per_task[tid] += 1

        # Consistency check: Schedule vs Config
        for tid, count in scheduled_robots_per_task.items():
            if tid in self.tasks_data:
                required = self.tasks_data[tid].get('numb_robots', 1)
                if count != required:
                    self.get_logger().error(f'CONSISTENCY ERROR: Task {tid} requires {required} robots but schedule has {count}!')
            else:
                self.get_logger().error(f'CONSISTENCY ERROR: Task {tid} in schedule NOT FOUND in tasks config!')

        self.get_logger().info(f'Loaded robots: {list(self.robot_queues.keys())}')
        for rid, q in self.robot_queues.items():
            self.get_logger().info(f'Robot {rid} has {len(q)} tasks in queue.')

        # tracked states
        self.robot_task_index = {rid: -1 for rid in self.schedule.keys()}
        self.ready_robots_per_task = {} 
        self.landing_sent = False
        self.warmup_duration = 5.0 # Seconds to wait before starting the clock
        self.last_republish_time = 0.0
        
        # Track which robots are assigned to which task
        self.task_assignments = {} # {tid: set(rids)}
        for rid, data in self.schedule.items():
            for tid in data['name']:
                if tid not in self.task_assignments:
                    self.task_assignments[tid] = set()
                self.task_assignments[tid].add(rid)

        # Dynamic scheduling states
        self.robot_arrivals = {tid: set() for tid in self.all_task_ids}
        self.task_start_time = {} # {tid: timestamp}
        self.task_status = {tid: 'PENDING' for tid in self.all_task_ids}
        
        # publisher for formation commands. This is for the consensus node :)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher = self.create_publisher(String, '/swarm_formation_cmd', qos_profile)
        
        # publisher for RViz markers.
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.marker_publisher = self.create_publisher(MarkerArray, '/tasks_markers', marker_qos)
        
        # active tasks tracking for visualization
        self.active_tasks = set()
        self.completed_tasks = set()
        
        # timer to check for transitions (10Hz for precision)
        self.start_time = self.get_clock().now()
        
        # Subscriber for robot status updates
        self.status_subscriber = self.create_subscription(
            String,
            '/swarm_task_status',
            self.status_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initial marker publish
        self.publish_markers()
        
        self.get_logger().info(f'Auction Manager (DIAGNOSTIC VERSION) started.')

    def load_tasks_config(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
                return {t['id']: t for t in data.get('tasks', [])}
        except Exception as e:
            self.get_logger().error(f'Failed to load tasks config: {e}')
            return {}

    def load_schedule(self, path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load schedule: {e}')
            return {}

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            tid = data.get('task_id')
            rid = data.get('robot_id')
            status = data.get('status')
            
            # Since robot IDs might be "cf_1" or "1" or "R1", we normalize
            rid_str = f"R{rid}" if isinstance(rid, int) else rid
            if not rid_str.startswith('R'):
                rid_str = rid_str.replace('cf_', 'R')

            if status == 'ARRIVED' and tid in self.robot_arrivals:
                if rid_str not in self.robot_arrivals[tid]:
                    self.robot_arrivals[tid].add(rid_str)
                    self.get_logger().info(f'Robot {rid_str} arrived at {tid}. Progress: {len(self.robot_arrivals[tid])}/{len(self.task_assignments.get(tid, []))}')
                
                # Check if all assigned robots arrived
                assigned = self.task_assignments.get(tid, set())
                if tid not in self.task_start_time and assigned.issubset(self.robot_arrivals[tid]):
                    self.task_start_time[tid] = self.get_clock().now()
                    self.get_logger().warn(f'TASK {tid} EXECUTION STARTED (all robots arrived).')
        except Exception as e:
            self.get_logger().error(f'Error in status callback: {e}')

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        if elapsed < self.warmup_duration:
            # Still warming up, just publish markers and wait
            if int(elapsed * 10) % 10 == 0: # Log every second
                self.get_logger().info(f'Warming up... Mission starts in {self.warmup_duration - elapsed:.1f}s', throttle_duration_sec=1.0)
            self.publish_markers()
            return

        current_sim_time = elapsed - self.warmup_duration
        now = self.get_clock().now()
        
        changes_made = False
        finished_all = True

        # 1. Update Completion Status based on duration
        for tid in list(self.active_tasks):
            if tid in self.task_start_time:
                duration = self.tasks_data.get(tid, {}).get('duration_s', 5.0)
                elapsed_task = (now - self.task_start_time[tid]).nanoseconds / 1e9
                if elapsed_task >= duration:
                    self.active_tasks.remove(tid)
                    self.completed_tasks.add(tid)
                    self.task_status[tid] = 'COMPLETED'
                    self.get_logger().info(f'TASK {tid} COMPLETED after {duration}s.')
                    changes_made = True

        # 2. Check for transitions
        new_task_assignments = {} # {task_id: [robot_ids]}

        for rid, queue in self.robot_queues.items():
            curr_idx = self.robot_task_index[rid]
            next_idx = curr_idx + 1
            
            # If current task is finished, we can move to next
            can_move_from_current = True
            if curr_idx >= 0:
                tid = queue[curr_idx]['task_id']
                if tid not in self.completed_tasks:
                    can_move_from_current = False
                    finished_all = False
            
            if next_idx < len(queue):
                finished_all = False
                if can_move_from_current:
                    next_tid = queue[next_idx]['task_id']
                    
                    # Check dependencies
                    deps = self.tasks_data.get(next_tid, {}).get('deps', [])
                    deps_met = all(dep in self.completed_tasks for dep in deps)
                    
                    if next_idx == 0:
                        self.get_logger().info(f'Robot {rid} first task: {next_tid}. Deps: {deps}', throttle_duration_sec=2.0)
                    
                    if deps_met:
                        # Move to next task!
                        self.get_logger().warn(f'Robot {rid} transitioning to {next_tid}')
                        self.robot_task_index[rid] = next_idx
                        tid = next_tid
                        
                        if tid not in self.ready_robots_per_task:
                            self.ready_robots_per_task[tid] = []
                        if rid not in self.ready_robots_per_task[tid]:
                            self.ready_robots_per_task[tid].append(rid)
                        
                        new_task_assignments[tid] = self.ready_robots_per_task[tid]
                        self.active_tasks.add(tid)
                        self.task_status[tid] = 'ACTIVE'
                        changes_made = True
            elif not can_move_from_current:
                finished_all = False
        
        # Publish only ONCE per task per transition
        for tid, rids in new_task_assignments.items():
            self.publish_robot_task(tid, rids)
        
        # Periodic republish (every 1.0s) of ALL active tasks to handle packet loss
        if current_sim_time - self.last_republish_time >= 1.0:
            self.last_republish_time = current_sim_time
            # Keep active tasks alive by republishing to anyone who missed it
            active_by_task = {} # tid -> [rids]
            for rid, queue in self.robot_queues.items():
                idx = self.robot_task_index[rid]
                if idx >= 0:
                    tid = queue[idx]['task_id']
                    if tid in self.active_tasks:
                        if tid not in active_by_task: active_by_task[tid] = []
                        active_by_task[tid].append(rid)
            
            for tid, rids in active_by_task.items():
                self.publish_robot_task(tid, rids)
            
            # Marker republish
            self.publish_markers()
        
        elif changes_made:
            self.publish_markers()
        
        if finished_all and not self.landing_sent:
            self.landing_sent = True
            self.get_logger().info('All tasks finished. Initiating landing sequence in 5s...')
            # Wait 5 seconds then send LAND command
            self.create_timer(5.0, self.send_landing_command)

    def send_landing_command(self):
        # Trigger LAND for all known robots
        robot_ids = [int(rid.replace('R','')) if isinstance(rid, str) else rid for rid in self.robot_queues.keys()]
        msg_dict = {
            'task_id': 'LAND',
            'robot_ids': robot_ids,
            'formation': 'LAND'
        }
        msg = String()
        msg.data = json.dumps(msg_dict)
        self.publisher.publish(msg)
        self.get_logger().warn('LAND command broadcasted. Shutting down in 5s...')
        
        # Final shutdown after landing starts
        self.create_timer(5.0, lambda: rclpy.shutdown())

    def resolve_schedule_path(self, arg):
        """This function is to solve some path issues within the container. Is not important"""
        if arg and os.path.isabs(arg) and os.path.exists(arg):
            return arg

        package_share = get_package_share_directory('mrs_auction')
        search_dirs = [
            os.getcwd(),                                     
            os.path.join(os.getcwd(), 'schedule'),           
            os.path.join(os.getcwd(), 'mrs_auction', 'schedule'), 
            os.path.join(package_share, 'mrs_auction', 'schedule'), # Package share path
            os.path.join(os.path.expanduser('~'), 'ros2_ws/src/mrs_auction/schedule'), # Home source dir
            '/root/ros2_ws/src/mrs_auction/schedule'          # Explicit Docker path
        ]

        if not arg or arg.lower() == 'latest':
            latest = self.get_latest_schedule(search_dirs)
            if latest:
                self.get_logger().info(f'Auto-detected latest schedule: {latest}')
                return latest
            else:
                self.get_logger().error('No schedule files found in search directories.')
                return None

        for d in search_dirs:
            p = os.path.join(d, arg)
            if os.path.exists(p):
                return p
        
        self.get_logger().error(f'Schedule file "{arg}" not found in search paths: {search_dirs}')
        return None

    def get_latest_schedule(self, directories):
        all_files = []
        for d in directories:
            if not os.path.exists(d):
                continue
            for f in os.listdir(d):
                if f.endswith('.json'):
                    all_files.append(os.path.join(d, f))
        
        if not all_files:
            return None
            
        # Return the one with the latest modification time.
        # Ensure we favor the most recent timestamp in filename if mtimes are weird
        all_files.sort(reverse=True) # Usually keeps filenames in order if they start with timestamp
        latest = max(all_files, key=os.path.getmtime)
        return latest

    def publish_robot_task(self, task_id, ready_robot_ids):
        #We publish the command for the task, but with the list of currently ready robots
        #This allows robots to start moving alone if their teammates are still busy
        task_config = self.tasks_data.get(task_id, {})
        location = task_config.get('location', [0.0, 0.0])
        formation = task_config.get('formation', 'line')
        spacing = task_config.get('formation_spacing', 1.0)
        
        # Convert IDs like "R1" to integer for consensus compatibility. I diidnt want to change Thi's code so I adapted the nomenclatire
        clean_ids = []
        for rid in ready_robot_ids:
            try:
                if isinstance(rid, str):
                    clean_id = rid.lower().replace('r', '')
                    clean_ids.append(int(clean_id))
                else:
                    clean_ids.append(int(rid))
            except:
                continue

        msg_dict = {
            'task_id': task_id,
            'robot_ids': clean_ids,
            'location': location,
            'formation': formation,
            'spacing': float(spacing)
        }
        
        msg = String()
        msg.data = json.dumps(msg_dict)
        self.publisher.publish(msg)
        self.get_logger().info(f'Task {task_id} updated for robots {clean_ids}.')

    def publish_markers(self):
        marker_array = MarkerArray()
        
        for i, tid in enumerate(self.all_task_ids):
            task_config = self.tasks_data[tid]
            loc = task_config.get('location', [0.0, 0.0])
            is_active = tid in self.active_tasks
            is_completed = tid in self.completed_tasks
            
            # cylinder marker for the task position
            marker = Marker()
            marker.header.frame_id = "world" 
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "task_positions"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(loc[0])
            marker.pose.position.y = float(loc[1])
            marker.pose.position.z = 0.3
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.6
            
            if is_completed:
                # BLUE for completed
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.5
                marker.pose.position.z = 0.05 # Flat on ground
                marker.scale.z = 0.1
            elif is_active:
                # GREEN for active
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.pose.position.z = 0.3
            else:
                # RED for pending
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                marker.pose.position.z = 0.3
            
            marker_array.markers.append(marker)
            
            # text label for the task ID
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "task_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(loc[0])
            text_marker.pose.position.y = float(loc[1])
            text_marker.pose.position.z = 1.2
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Task:{tid}"
            # if is_active:
            #     text_marker.text += " (ACTIVE)"
            
            marker_array.markers.append(text_marker)
            
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AuctionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
