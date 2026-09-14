#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    arm_pkg_path = FindPackageShare('hex_ros_robot_arm')
    traj_pkg_path = FindPackageShare('hex_ros_demo_arm_replay')

    # args
    robot_host_arg = DeclareLaunchArgument(
        name='robot_host',
        default_value='172.18.20.80',
        description='Robot controller IP address')
    robot_port_arg = DeclareLaunchArgument(
        name='robot_port',
        default_value='8439',
        description='Robot controller WebSocket port')
    robot_grip_type_arg = DeclareLaunchArgument(
        name='robot_grip_type',
        default_value='empty',
        choices=['gp100', 'gp80', 'gr100', 'empty'],
        description='Grip type: gp100/gp80/gr100 (1-DoF) or empty (0-DoF)')
    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value='firefly',
        choices=['archer', 'firefly'],
        description='Robot arm type: archer or firefly')
    enable_keyboard_arg = DeclareLaunchArgument(
        name='enable_keyboard',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to launch the keyboard teleoperation node')

    # robot launch file name: "archer.launch.py" / "firefly.launch.py"
    robot_launch_file = PythonExpression(
        ['"', LaunchConfiguration('robot_type'), '.launch.py"'])

    # keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])),
        condition=IfCondition(LaunchConfiguration('enable_keyboard')),
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [arm_pkg_path, robot_launch_file])),
        launch_arguments={
            'robot_host': LaunchConfiguration('robot_host'),
            'robot_port': LaunchConfiguration('robot_port'),
            'robot_grip_type': LaunchConfiguration('robot_grip_type'),
            'test': 'false',
        }.items(),
    )


    # arm trajectory node
    traj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([traj_pkg_path, "arm_replay.launch.py"])),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        robot_host_arg,
        robot_port_arg,
        robot_grip_type_arg,
        robot_type_arg,
        enable_keyboard_arg,
        keyboard_launch,
        arm_launch,
        traj_launch,
    ])
