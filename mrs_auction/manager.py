#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
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
            tasks_config_path = os.path.join(package_share, 'config', tasks_config_path if tasks_config_path else 'tasks.yaml')
        
        if not os.path.exists(tasks_config_path):
            self.get_logger().error(f'Tasks config not found: {tasks_config_path}')
            return

        # Load tasks configuration to get locations and formations
        self.tasks_data = self.load_tasks_config(tasks_config_path)
        
        schedule_path = self.resolve_schedule_path(schedule_arg)
        if not schedule_path:
            return
        
        # Load schedule
        self.schedule = self.load_schedule(schedule_path)
        
        # flatten schedule into per-robot queues. Its how we discussed it. Each robot has a queue of tasks
        self.robot_queues = {rid: [] for rid in self.schedule.keys()}
        for rid, data in self.schedule.items():
            for i in range(len(data['name'])):
                self.robot_queues[rid].append({
                    'task_id': data['name'][i],
                    'start_time': data['time'][i][0],
                    'finish_time': data['time'][i][1]
                })

        # tracked states
        self.robot_task_index = {rid: -1 for rid in self.schedule.keys()}
        self.ready_robots_per_task = {} 
        self.landing_sent = False
        
        # publisher for formation commands. This is for the consensus node :)
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher = self.create_publisher(String, '/swarm_formation_cmd', qos_profile)
        
        # publisher for RViz markers. I think it looks cool but you guys tell me if you like it
        self.marker_publisher = self.create_publisher(MarkerArray, '/tasks_markers', 10)
        
        # active tasks tracking for visualization
        self.active_tasks = set()
        self.completed_tasks = set()
        self.all_task_ids = list(self.tasks_data.keys())
        
        # timer to check for transitions (10Hz for precision)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initial marker publish
        self.publish_markers()
        
        self.get_logger().info(f'Auction Manager started. Robots will start moving to their first tasks.')

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

    def timer_callback(self):
        current_sim_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        changes_made = False
        finished_all = True

        for rid, queue in self.robot_queues.items():
            curr_idx = self.robot_task_index[rid]
            next_idx = curr_idx + 1
            
            # Check if current task finished
            if curr_idx >= 0:
                curr_task = queue[curr_idx]
                if current_sim_time >= curr_task['finish_time']:
                    # robot has finished this task's work time
                    tid = curr_task['task_id']
                    if tid in self.ready_robots_per_task and rid in self.ready_robots_per_task[tid]:
                        self.ready_robots_per_task[tid].remove(rid)
                        if len(self.ready_robots_per_task[tid]) == 0:
                            if tid in self.active_tasks:
                                self.active_tasks.remove(tid)
                            self.completed_tasks.add(tid)
                            changes_made = True
                else:
                    finished_all = False
            
            #check if we should start the next task
            if next_idx < len(queue):
                finished_all = False
                # start the first task at t=0, or subsequent tasks when the previous one finishes
                dispatch_time = 0.0 if next_idx == 0 else queue[curr_idx]['finish_time']
                
                if current_sim_time >= dispatch_time:
                    # Move to next task
                    self.robot_task_index[rid] = next_idx
                    next_task = queue[next_idx]
                    tid = next_task['task_id']
                    
                    # Update ready robots for this task
                    if tid not in self.ready_robots_per_task:
                        self.ready_robots_per_task[tid] = []
                    if rid not in self.ready_robots_per_task[tid]:
                        self.ready_robots_per_task[tid].append(rid)
                    
                    self.publish_robot_task(tid, self.ready_robots_per_task[tid])
                    self.active_tasks.add(tid)

                    #safety check
                    if tid in self.completed_tasks:
                        self.completed_tasks.remove(tid)
                    changes_made = True
        
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
            
        #Return the one with latest modification time
        return max(all_files, key=os.path.getmtime)

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
