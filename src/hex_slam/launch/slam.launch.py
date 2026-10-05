#!/usr/bin/env python3
"""Bring up SLAM for the hex2 hexapod.

MODULAR ON PURPOSE: this launch file does NOT start Gazebo, robot_state_publisher, or
ros2_control - those must already be running via `ros2 launch hex_gazebo gazebo.launch.py`
in a separate terminal first. This file only adds nodes downstream of the existing,
unmodified /scan, /imu, /odom, and TF tree:

  ekf_filter_node (robot_localization) : fuses /odom (gz ground truth) + /imu
                                          -> publishes odom -> base_footprint TF
  slam_toolbox (online_async)          : consumes /scan + that TF tree
                                          -> publishes map -> odom TF and /map

Default (rviz:=false): starts no RViz of its own. Point an already-open RViz
(use_sim_time:=true) at TF + /scan + /map and it updates live as these nodes
publish. Pass rviz:=true to spawn a separate RViz instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hex_slam_share = get_package_share_directory('hex_slam')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    ekf_config = os.path.join(hex_slam_share, 'config', 'ekf.yaml')
    slam_params = os.path.join(hex_slam_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(hex_slam_share, 'rviz', 'slam.rviz')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch own RViz instance. Default off: assumes an already-open '
                     'RViz (e.g. from hex_description display.launch.py) with '
                     'use_sim_time:=true, subscribed to /scan, /map, TF - this launch '
                     'file only adds ekf_filter_node + slam_toolbox so that RViz '
                     'picks up map/TF updates live. Set true to spawn a separate '
                     'RViz with hex_slam/rviz/slam.rviz instead.',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='hex_slam_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_rviz,
        ekf_node,
        slam_toolbox_launch,
        rviz_node,
    ])
