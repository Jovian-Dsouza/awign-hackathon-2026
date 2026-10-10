"""rosbridge + static laser TF + slam_toolbox (online sync), all on simulator time."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PARAMS = "/qcw/config/slam_toolbox.yaml"
# Stretch 3 RPLidar: base_link -> laser (Menagerie `laser` body: pos 0.004 0 0.1664, yawed 180 deg).
LASER_TF = ["--x", "0.004", "--y", "0", "--z", "0.1664", "--yaw", "3.141592653589793",
            "--frame-id", "base_link", "--child-frame-id", "laser"]


def generate_launch_description() -> LaunchDescription:
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_sync_launch.py")
    return LaunchDescription([
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            parameters=[{"port": 9090, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_tf",
            arguments=LASER_TF,
            parameters=[{"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"slam_params_file": PARAMS, "use_sim_time": "true"}.items(),
        ),
    ])
