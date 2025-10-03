#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default='true')

    # robot description (URDF) - publish via robot_state_publisher
    pkg_share = FindPackageShare(package='self_balancer_nodes').find('self_balancer_nodes')
    urdf_file = os.path.join(pkg_share, '..', 'urdf', 'self_balancer.urdf.xacro')

    # spawn robot in Gazebo - user should have world with a flat ground and small wrench plugins
    # IMU in gazebo should publish /sim_imu (user must set Gazebo imu plugin to this topic, or remap)
    nodes = [
        Node(
            package='self_balancer_nodes',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'use_sim': True, 'sim_imu_topic': '/imu'}]
        ),
        Node(
            package='self_balancer_nodes',
            executable='balance_controller',
            name='balance_controller',
            output='screen',
            parameters=[{'dt': 0.01}]
        ),
        Node(
            package='self_balancer_nodes',
            executable='motor_comm',
            name='motor_comm',
            output='screen',
            parameters=[{'use_sim': True}]
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true'),
        *nodes
    ])
