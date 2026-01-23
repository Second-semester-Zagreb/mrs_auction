from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("mrs_auction")
    run_auction = ExecuteProcess(
        cmd=["python3", os.path.join(pkg_share, "scripts", "run_auction.py")],
        output="screen",
    )

    launch_execute = RegisterEventHandler(
        OnProcessExit(
            target_action=run_auction,
            on_exit=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "execute_schedule.launch.py")
                )
            )],
        )
    )

    return LaunchDescription([run_auction, launch_execute])