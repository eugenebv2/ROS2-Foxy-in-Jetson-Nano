#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Default YAML file path (can be overridden at launch)
    default_yaml = os.path.join(
        os.path.expanduser('~'),
        'ros2_lc32/sg_yaml/servo_commands.yaml'  # Change to your actual YAML file path
    )

    return LaunchDescription([
        # Allow user to pass YAML path at launch
        DeclareLaunchArgument('yaml_path', default_value=default_yaml, description='Path to YAML file'),
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1', description='Serial port for LSC-32'),
        DeclareLaunchArgument('baudrate', default_value='9600', description='Baud rate for LSC-32'),
        #DeclareLaunchArgument(
        #    'yaml_path',
        #    default_value=default_yaml,
        #    description='Path to YAML file with servo commands'
        #),

        # LSC-32 multi-servo controller node
        Node(
            package='arm_test',  # Replace with your package name
            executable='lsc32_multi_servo_controller_param',
            name='lsc32_multi_servo_controller',
            output='screen',
            parameters=[
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
                ]
        ),

        # LSC-32 command publisher node
        Node(
            package='arm_test',  # Replace with your package name
            executable='lsc32_command_publisher',
            name='lsc32_command_publisher',
            output='screen',
            arguments=[LaunchConfiguration('yaml_path')]
        )
    ])
