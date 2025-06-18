from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory("ez_analog_processor")

    # Define config file paths
    sensors_config = os.path.join(pkg_share, "config", "sensors.yaml")
    aws_config = os.path.join(pkg_share, "config", "aws_iot.yaml")

    return LaunchDescription([
        # Launch the analog processor node first
        Node(
            package="ez_analog_processor",
            executable="analog_processor",
            name="analog_processor",
            parameters=[sensors_config],
            output="screen",
        ),

        # Launch the AWS publisher node
        Node(
            package="ez_analog_processor",
            executable="publish_to_AWS",
            name="aws_publisher",
            parameters=[aws_config],
            output="screen",
        ),
    ])
