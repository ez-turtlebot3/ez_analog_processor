from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ez_analog_processor'),
        'config',
        'sensors.yaml'
    )

    return LaunchDescription([
        Node(
            package='ez_analog_processor',
            executable='analog_processor',
            name='analog_processor',
            parameters=[config_file],
            output='screen'
        )
    ])
