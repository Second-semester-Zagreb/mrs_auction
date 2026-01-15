#!/usr/bin/env python3
import os
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load latest schedule
    schedule_dir = "/root/ros2_ws/src/mrs_auction/schedule"
    schedule_files = sorted([f for f in os.listdir(schedule_dir) if f.startswith("schedule_") and f.endswith(".json")])
    
    if not schedule_files:
        raise RuntimeError("No schedule found! Run auction first.")
    
    latest_schedule = os.path.join(schedule_dir, schedule_files[-1])
    with open(latest_schedule, "r") as f:
        sol = json.load(f)
    
    print(f"Loading schedule: {latest_schedule}")
    print(f"Robots: {list(sol.keys())}")
    
    nodes = []
    for robot_id in sol.keys():
        nodes.append(
            Node(
                package='mrs_auction',
                executable='simple_executor_node',
                name=f'executor_{robot_id}',
                output='screen',
                arguments=[robot_id],
                parameters=[{
                    'arrival_tol': 0.15,
                    'k_p': 0.8,
                    'max_vel': 0.25,
                    'rate_hz': 20.0,
                    'align_to_now': True,
                    'cmd_in_body_frame': True,
                }]
            )
        )
    
    return LaunchDescription(nodes)
