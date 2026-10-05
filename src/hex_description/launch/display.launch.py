#!/usr/bin/env python3
"""Launch RViz2 with robot_state_publisher + joint_state_publisher_gui for hex_description."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hex_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'hex_description.rviz')

    # Default slider values = STAND pose (same on every leg), because base_footprint_joint's
    # z offset assumes it. Single source of truth: config/rest_pose.yaml (also feeds the xacro).
    with open(os.path.join(pkg_share, 'config', 'rest_pose.yaml')) as f:
        pose = yaml.safe_load(f)['stand']
    zeros = {}
    for i in range(1, 7):
        zeros[f'leg{i}_coxa_joint'] = float(pose['coxa'])
        zeros[f'leg{i}_femur_joint'] = float(pose['femur'])
        zeros[f'leg{i}_tibia_joint'] = float(pose['tibia'])

    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui for manual joint sliders.',
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 config file.',
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
        parameters=[{'zeros': zeros}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_use_gui,
        declare_rviz_config,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])