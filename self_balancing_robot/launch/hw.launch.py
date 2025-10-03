#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    baudrate = LaunchConfiguration('baudrate', default='115200')

    imu_bridge = Node(
        package='self_balancer_nodes',
        executable='imu_bridge',
        output='screen',
        parameters=[{'use_sim': False, 'serial_port': serial_port, 'baudrate': int(baudrate)}]
    )

    motor_comm = Node(
        package='self_balancer_nodes',
        executable='motor_comm',
        output='screen',
        parameters=[{'use_sim': False, 'serial_port': serial_port, 'baudrate': int(baudrate)}]
    )

    controller = Node(
        package='self_balancer_nodes',
        executable='balance_controller',
        output='screen',
        parameters=[{'dt': 0.005, 'kp': 80.0, 'kd': 1.5, 'max_output': 120.0}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        imu_bridge, motor_comm, controller
    ])
