#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    base_launch = os.path.join(
        pkg_tb3_gazebo, 'launch', 'nav2_aws_warehouse.launch.py'
    )
    default_map = os.path.join(
        pkg_tb3_gazebo, 'maps', 'warehouse_v2', 'map.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    headless = LaunchConfiguration('headless', default='false')
    no_roof = LaunchConfiguration('no_roof', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    slam = LaunchConfiguration('slam', default='false')

    launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'headless': headless,
            'no_roof': no_roof,
            'use_rviz': use_rviz,
            'slam': slam,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=default_map))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-3.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('no_roof', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='false'))
    ld.add_action(launch_cmd)
    return ld
