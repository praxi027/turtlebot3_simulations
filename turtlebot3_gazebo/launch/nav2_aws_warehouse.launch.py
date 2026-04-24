#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_aws_warehouse = get_package_share_directory(
        'aws_robomaker_small_warehouse_world'
    )

    default_map = os.path.join(
        pkg_aws_warehouse, 'maps', '005', 'map.yaml'
    )
    default_params = os.path.join(
        pkg_tb3_gazebo, 'params', 'nav2_aws_warehouse.yaml'
    )
    default_rviz = os.path.join(
        pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    # Renamed from `params_file` to avoid collision with gzserver.launch.py's
    # own `params_file` LaunchConfiguration (it would otherwise feed the Nav2
    # yaml to gzserver and crash it).
    nav2_params = LaunchConfiguration('nav2_params_file', default=default_params)
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    headless = LaunchConfiguration('headless', default='false')
    no_roof = LaunchConfiguration('no_roof', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz)
    # When true, run slam_toolbox online instead of AMCL + static map
    # (nav2_bringup swaps localization_launch.py for slam_launch.py).
    slam = LaunchConfiguration('slam', default='false')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_aws_warehouse.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'headless': headless,
            'no_roof': no_roof,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Scope the Nav2 include so its `params_file` / `map` LaunchConfigurations
    # don't leak back into gzserver's namespace.
    # Two branches because nav2_bringup's bringup_launch.py expects Python-style
    # 'True'/'False' for `slam` (it's fed to PythonExpression), but IfCondition
    # accepts lowercase `true`/`false` too — so users can pass either.
    nav2_slam_cmd = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'True',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
            }.items(),
        ),
    ], condition=IfCondition(slam))

    nav2_amcl_cmd = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
            }.items(),
        ),
    ], condition=UnlessCondition(slam))

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=default_map))
    ld.add_action(DeclareLaunchArgument('nav2_params_file', default_value=default_params))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('no_roof', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=default_rviz))
    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='false',
        description='Run slam_toolbox online (no prior map) instead of AMCL.'
    ))
    ld.add_action(gazebo_cmd)
    ld.add_action(nav2_slam_cmd)
    ld.add_action(nav2_amcl_cmd)
    ld.add_action(rviz_cmd)
    return ld
