#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    traj_pkg_path = FindPackageShare('hex_ros_demo_arm_replay')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_archer_y6')

    # args
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to use sim time')

    # arm_replay node
    traj_param_path = PathJoinSubstitution(
        [traj_pkg_path, "config", "ros2", "replay_param.yaml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "gr100_comp.urdf"])

    arm_replay_node = Node(
        package='hex_ros_demo_arm_replay',
        executable='arm_replay',
        name='arm_replay',
        output="screen",
        emulate_tty=True,
        parameters=[
            traj_param_path,
            {
                "model_urdf": ParameterValue(urdf_file_path, value_type=str),
                "use_sim_time": LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('manip_state', 'manip_state'),
            ('manip_ctrl', 'manip_ctrl'),
            ('teleop_keyboard_state', 'teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        arm_replay_node,
    ])
