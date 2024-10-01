from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "foxglove",
                default_value="false",
                choices=["true", "false"],
                description="Whether to launch the Foxglove bridge",
            ),
            Node(
                package="simplesim",
                executable="simplesim_node",
                output="screen",
            ),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory(
                                "foxglove_bridge", print_warning=True
                            ),
                            "launch/foxglove_bridge_launch.xml",
                        )
                    ]
                ),
                condition=IfCondition(LaunchConfiguration("foxglove")),
            ),
        ]
    )
