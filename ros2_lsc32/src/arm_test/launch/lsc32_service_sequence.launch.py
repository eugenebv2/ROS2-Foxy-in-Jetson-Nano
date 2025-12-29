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
        'ros2_lc32/sg_yaml/servo_sequence.yaml'  # Change to your actual YAML file path
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'yaml_path',
            default_value=default_yaml,
            description='Path to YAML file with servo commands'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyTHS1',
            description='Serial port for LSC-32'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='9600',
            description='Baud rate for LSC-32'
        ),

        # Service server node
        Node(
            package='arm_test',  # Replace with your package name
            executable='lsc32_multi_servo_service',
            name='lsc32_multi_servo_service',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate')
            }]
        ),

        # YAML-driven service client node
        Node(
            package='arm_test',  # Replace with your package name
            executable='lsc32_yaml_service_client',
            name='lsc32_yaml_service_client',
            output='screen',
            arguments=[LaunchConfiguration('yaml_path')]
        )
    ])

