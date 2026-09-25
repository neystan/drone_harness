import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """仅启动 AirSim 相机桥接，避免加载已移除的追踪预览链。"""
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("airsim_ros_pkgs"),
                "launch",
                "airsim_node.launch.py",
            )
        )
    )

    return LaunchDescription([airsim_node_launch])
