import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """启动 AirSim 相机桥接和相机预览节点。"""
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("airsim_ros_pkgs"),
                "launch",
                "airsim_node.launch.py",
            )
        )
    )

    preview_node = Node(
        package="drone_agent",
        executable="camera_view_sim",
        name="camera_view_sim",
        output="screen",
    )

    return LaunchDescription([airsim_node_launch, preview_node])
