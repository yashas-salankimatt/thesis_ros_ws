#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    # robot moveit fake launch
    # xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    robot_moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_robot_moveit_fake.launch.py'])),
        launch_arguments={
            'dof': '7',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
        }.items(),
    )

    # start the twist publisher node from the dimensional package
    twist_publisher_launch = Node(
        package='dimensional',
        executable='tf_to_twist_publisher',
        name='tf_to_twist_publisher',
        output='screen',
    )

    # tf_to_twist_publisher with parent frame 'odom' and child frame 'xarm_link_7'
    alfred_base_center_twist_publisher_launch = Node(
        package='dimensional',
        executable='tf_to_twist_publisher',
        name='tf_to_twist_publisher',
        output='screen',
        parameters=[
            {'parent_frame': 'odom'},
            {'child_frame': 'base_center'},
            {'twist_topic': '/alfred_base_center_tf_twist'},
        ]
    )

    alfred_controller_launch = Node(
        package='dimensional',
        executable='alfred_controller',
        name='alfred_controller',
        output='screen',
    )
    
    return LaunchDescription([
        robot_moveit_fake_launch,
        twist_publisher_launch,
        alfred_base_center_twist_publisher_launch,
        alfred_controller_launch
    ])
