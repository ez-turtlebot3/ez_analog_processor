from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory("ez_tb3_streamer")

    # Declare the launch arguments
    aws_endpoint = LaunchConfiguration("aws_endpoint")

    # Create the launch description
    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument("aws_endpoint", description="AWS IoT Core endpoint"),
            # Launch the AWS publisher node
            Node(
                package="ez_tb3_streamer",
                executable="publish_to_AWS",
                name="aws_publisher",
                parameters=[
                    os.path.join(pkg_share, "config", "sensors.yaml"),
                    os.path.join(pkg_share, "config", "aws_iot.yaml"),
                    {"aws_iot.endpoint": aws_endpoint},
                ],
                output="screen",
            ),
        ]
    )
