#!/usr/bin/env python3
"""Launch Gazebo Sim (gz-sim) with the hexapod spawned via ros2_control + gz_ros2_control.

Reuses hex_description/urdf/robot.xacro (through hex_gazebo/urdf/hex.gazebo.xacro, which
only adds <gazebo>/<ros2_control> tags on top of it) so there is exactly one model source.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hex_gazebo_share = get_package_share_directory('hex_gazebo')
    hex_description_share = get_package_share_directory('hex_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        hex_gazebo_share,
        'urdf',
        'hex.gazebo.xacro'
    )

    bridge_config = os.path.join(
        hex_gazebo_share,
        'config',
        'ros_gz_bridge.yaml'
    )

    # -------------------------------------------------------------------------
    # Gazebo resource path
    #
    # Allows Gazebo to resolve:
    #
    #   model://hex_description/meshes/*.stl
    #
    # IMPORTANT:
    # get_package_share_directory('hex_description') returns:
    #
    #   .../install/hex_description/share/hex_description
    #
    # Gazebo needs the parent "share" directory:
    #
    #   .../install/hex_description/share
    #
    # so that:
    #
    #   model://hex_description/meshes/base_link.stl
    #
    # resolves to:
    #
    #   .../share/hex_description/meshes/base_link.stl
    # -------------------------------------------------------------------------

    hex_description_resource_path = os.path.dirname(
        hex_description_share
    )

    existing_gz_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    if existing_gz_resource_path:
        gz_resource_path = (
            hex_description_resource_path
            + os.pathsep
            + existing_gz_resource_path
        )
    else:
        gz_resource_path = hex_description_resource_path

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path,
    )

    # -------------------------------------------------------------------------
    # World
    # -------------------------------------------------------------------------

    world = LaunchConfiguration('world')

    room_world = os.path.join(
        hex_gazebo_share,
        'worlds',
        'room.sdf'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=room_world,
        description='Gazebo Sim world file to load.',
    )

    # -------------------------------------------------------------------------
    # Robot description
    # -------------------------------------------------------------------------

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # -------------------------------------------------------------------------
    # Gazebo
    # -------------------------------------------------------------------------

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world]
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Robot State Publisher
    # -------------------------------------------------------------------------

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Spawn hexapod
    # -------------------------------------------------------------------------

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hexapod',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'hexapod',
            '-z',
            '0.08',
        ],
    )

    # -------------------------------------------------------------------------
    # ROS-Gazebo bridge
    # -------------------------------------------------------------------------

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hex_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
                'use_sim_time': True
            }
        ],
    )

    # -------------------------------------------------------------------------
    # Controllers
    # -------------------------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hex_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen',
    )

    # Fire controller spawners once spawn_entity has finished (i.e. the model, and with it
    # gz_ros2_control's /controller_manager, exists in the running Gazebo server), instead
    # of guessing a fixed delay. NOTE: spawn_entity exiting confirms the entity was created;
    # if /controller_manager takes noticeably longer to come up than that on a given machine,
    # the spawner calls below will still retry/fail per controller_manager spawner's own
    # timeout - this removes the arbitrary-5s race but does not add an explicit service-wait.
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                trajectory_controller_spawner,
            ],
        )
    )

    # -------------------------------------------------------------------------
    # Launch
    # -------------------------------------------------------------------------

    return LaunchDescription([
        declare_world,

        # IMPORTANT:
        # Set this BEFORE Gazebo starts.
        gazebo_resource_path,

        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        ros_gz_bridge,
        delayed_controller_spawners,
    ])