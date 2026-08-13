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
    robot_type_arg = DeclareLaunchArgument(
        name='robot_type',
        default_value='archer',
        choices=['archer', 'firefly'],
        description='Robot arm type: archer or firefly')

    # robot launch file name: "archer.launch.py" / "firefly.launch.py"
    robot_launch_file = PythonExpression(
        ['"', LaunchConfiguration('robot_type'), '.launch.py"'])

    # keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])), )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [arm_pkg_path, robot_launch_file])), )


    # arm trajectory node
    traj_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([traj_pkg_path, "arm_replay.launch.py"])),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        robot_type_arg,
        keyboard_launch,
        arm_launch,
        traj_launch,
    ])
