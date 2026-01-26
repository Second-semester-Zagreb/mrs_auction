import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'mrs_auction'
    
    # Declare launch arguments
    schedule_file_arg = DeclareLaunchArgument(
        'schedule_file',
        default_value='latest',
        description='Full path to the schedule JSON file (or "latest" to auto-detect)'
    )
    
    tasks_config_arg = DeclareLaunchArgument(
        'tasks_config',
        default_value=os.path.join(get_package_share_directory(package_name), 'config', 'custom_tasks.yaml'),
        description='Full path to the tasks configuration YAML file'
    )

    manager_node = Node(
        package=package_name,
        executable='manager_node',
        name='manager_node',
        output='screen',
        parameters=[{
            'schedule_file': LaunchConfiguration('schedule_file'),
            'tasks_config': LaunchConfiguration('tasks_config'),
        }]
    )

    return LaunchDescription([
        schedule_file_arg,
        tasks_config_arg,
        manager_node
    ])
