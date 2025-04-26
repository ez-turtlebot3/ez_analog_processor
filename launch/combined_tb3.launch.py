#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    ez_tb3_streamer_dir = get_package_share_directory('ez_tb3_streamer')
    
    # Create launch configuration variables
    namespace = LaunchConfiguration('namespace', default='')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=namespace,
        description='Namespace for nodes')
        
    declare_usb_port_cmd = DeclareLaunchArgument(
        'usb_port',
        default_value=usb_port,
        description='Connected USB port with OpenCR')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')
    
    # Include the robot.launch.py file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'usb_port': usb_port,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include the analog_processor.launch.py
    analog_processor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ez_tb3_streamer_dir, 'launch', 'analog_processor.launch.py')
        )
    )
    
    # Use the turtlebot3_node as a trigger for the analog processor launch
    # This ensures the analog processor launches after the robot is up
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        output='screen',
        arguments=['-i', usb_port],
    )
    
    # Register event handler to wait for the turtlebot3_node to start before launching analog_processor
    analog_processor_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtlebot3_node,
            on_start=[analog_processor_launch]
        )
    )
    
    # Create the launch description with sequential ordering
    return LaunchDescription([
        declare_namespace_cmd,
        declare_usb_port_cmd,
        declare_use_sim_time_cmd,
        robot_launch,
        analog_processor_event
    ])