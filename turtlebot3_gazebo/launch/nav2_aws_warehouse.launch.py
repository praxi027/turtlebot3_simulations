#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    default_map = os.path.join(
        pkg_tb3_gazebo, 'maps', 'warehouse', 'map.yaml'
    )
    default_keepout_mask = os.path.join(
        pkg_tb3_gazebo, 'maps', 'warehouse', 'keepout_mask.yaml'
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
    keepout_mask = LaunchConfiguration('keepout_mask', default=default_keepout_mask)
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    gazebo_port = LaunchConfiguration('gazebo_port', default='11345')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    headless = LaunchConfiguration('headless', default='false')
    no_roof = LaunchConfiguration('no_roof', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz)
    autostart = LaunchConfiguration('autostart', default='true')
    # When true, run slam_toolbox online instead of AMCL + static map
    # (nav2_bringup swaps localization_launch.py for slam_launch.py).
    slam = LaunchConfiguration('slam', default='false')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x': initial_pose_x,
            'amcl.ros__parameters.initial_pose.y': initial_pose_y,
            'amcl.ros__parameters.initial_pose.z': initial_pose_z,
            'amcl.ros__parameters.initial_pose.yaw': initial_pose_yaw,
        },
        convert_types=True,
    )
    # Fast DDS shared-memory transport is flaky on this host and can stall the
    # Gazebo/ROS factory path; force UDPv4 transport for warehouse launches.
    set_fastdds_builtin_transports = SetEnvironmentVariable(
        name='FASTDDS_BUILTIN_TRANSPORTS',
        value='UDPv4',
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_aws_warehouse.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'gazebo_port': gazebo_port,
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
    nav2_slam_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'True',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': configured_nav2_params,
                'autostart': autostart,
            }.items(),
        ),
    ], condition=IfCondition(slam))
    nav2_slam_cmd = TimerAction(
        period=7.0,
        actions=[nav2_slam_group],
    )

    nav2_amcl_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': configured_nav2_params,
                'autostart': autostart,
            }.items(),
        ),
    ], condition=UnlessCondition(slam))
    nav2_amcl_cmd = TimerAction(
        period=7.0,
        actions=[nav2_amcl_group],
    )

    filter_mask_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': keepout_mask,
            'topic_name': 'filter_mask',
            'frame_id': 'map',
        }],
    )

    costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/filter_mask',
            'type': 0,
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    costmap_filter_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )
    # The filter nodes sometimes need a beat to finish creating their lifecycle
    # services; starting the manager immediately can intermittently fail the
    # first configure transition before the nodes finish their own startup.
    costmap_filter_lifecycle_cmd = TimerAction(
        period=2.0,
        actions=[costmap_filter_lifecycle_node],
    )

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
    ld.add_action(DeclareLaunchArgument('keepout_mask', default_value=default_keepout_mask))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('gazebo_port', default_value='11345'))
    ld.add_action(DeclareLaunchArgument('initial_pose_x', default_value=x_pose))
    ld.add_action(DeclareLaunchArgument('initial_pose_y', default_value=y_pose))
    ld.add_action(DeclareLaunchArgument('initial_pose_z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('initial_pose_yaw', default_value=yaw))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('no_roof', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=default_rviz))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'slam', default_value='false',
        description='Run slam_toolbox online (no prior map) instead of AMCL.'
    ))
    ld.add_action(set_fastdds_builtin_transports)
    ld.add_action(gazebo_cmd)
    ld.add_action(filter_mask_server_cmd)
    ld.add_action(costmap_filter_info_server_cmd)
    ld.add_action(costmap_filter_lifecycle_cmd)
    ld.add_action(nav2_slam_cmd)
    ld.add_action(nav2_amcl_cmd)
    ld.add_action(rviz_cmd)
    return ld
