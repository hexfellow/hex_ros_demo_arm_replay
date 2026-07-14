#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-06-30
################################################################

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    arm_pkg_path = FindPackageShare('hex_ros_robot_arm')
    comp_pkg_path = FindPackageShare('hex_ros_arm_comp')

    # args

    # keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])), )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [arm_pkg_path, "archer.launch.py"])), )


    # gravity compensation node
    comp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([comp_pkg_path, "arm_comp.launch.py"])), )

    return LaunchDescription([
        keyboard_launch,
        arm_launch,
        comp_launch,
    ])
