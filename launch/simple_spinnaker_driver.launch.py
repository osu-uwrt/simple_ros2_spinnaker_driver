import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value = os.path.join(get_package_share_directory("simple_ros2_spinnaker_driver"), "config", "config.yaml"),
            description = "The path to the config file to use"
        ),
        
        Node(
            package="simple_ros2_spinnaker_driver",
            executable="spinnaker_node",
            name="spinnaker_node",
            output="screen",
            parameters=[
                LC("config")
            ]
        )
    ])
