import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_description = []
    number_of_robots = int(os.environ.get("NUM_ROBOTS", "2"))

    launch_description.append(
        SetEnvironmentVariable(name="NUM_ROBOTS", value=str(number_of_robots))
    )

    # ---------- Consensus nodes ----------
    consensus_cfg = os.path.join(
        get_package_share_directory("consensus"),
        "config",
        "config.yaml",
    )

    for i in range(1, number_of_robots + 1):
        launch_description.append(
            Node(
                package="consensus",
                executable="consensus_node",
                name=f"consensus_node_{i}",
                output="screen",
                parameters=[
                    consensus_cfg,
                    {
                        "robot_id": i,
                        "number_of_robots": number_of_robots,
                    },
                ],
            )
        )

    launch_description.append(
        Node(
            package="mrs_auction",
            executable="mission_executor_node",
            name="mission_executor_node",
            output="screen",
            parameters=[
                {
                    "arrival_tol": 0.15,
                    "k_p": 0.8,
                    "max_vel": 0.2,
                    "rate_hz": 20.0,
                    "align_to_now": True,
                    "sr_robot_id": 2,
                }
            ],
        )
    )

    return LaunchDescription(launch_description)
