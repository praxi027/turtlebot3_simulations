#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_aws_warehouse = get_package_share_directory(
        'aws_robomaker_small_warehouse_world'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    gazebo_port = LaunchConfiguration('gazebo_port', default='11345')
    headless = LaunchConfiguration('headless', default='false')
    no_roof = LaunchConfiguration('no_roof', default='true')

    world_roof = os.path.join(
        pkg_aws_warehouse, 'worlds', 'small_warehouse', 'small_warehouse.world'
    )
    world_no_roof = os.path.join(
        pkg_aws_warehouse, 'worlds',
        'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
    )

    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_path = os.path.join(
        pkg_tb3_gazebo, 'models',
        'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'
    )

    # Make Gazebo Classic find both TurtleBot3 and AWS warehouse models.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(pkg_tb3_gazebo, 'models'), ':',
            os.path.join(pkg_aws_warehouse, 'models'), ':',
            os.environ.get('GAZEBO_MODEL_PATH', ''),
        ],
    )
    set_gazebo_master_uri = SetEnvironmentVariable(
        name='GAZEBO_MASTER_URI',
        value=['http://127.0.0.1:', gazebo_port],
    )
    set_fastdds_builtin_transports = SetEnvironmentVariable(
        name='FASTDDS_BUILTIN_TRANSPORTS',
        value='UDPv4',
    )

    gzserver_cmd_no_roof = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_no_roof}.items(),
        condition=IfCondition(no_roof),
    )
    gzserver_cmd_roof = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_roof}.items(),
        condition=UnlessCondition(no_roof),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=UnlessCondition(headless),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw,
        ],
        output='screen',
    )
    # The Gazebo factory service can appear before the warehouse world is fully
    # ready to accept the robot model; a short delay avoids hanging the initial
    # burger spawn while later pedestrian spawns still work.
    spawn_turtlebot_after_world = TimerAction(
        period=5.0,
        actions=[spawn_turtlebot_cmd],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument(
        'gazebo_port', default_value='11345',
        description='Gazebo master TCP port for this warehouse instance.'
    ))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'no_roof', default_value='true',
        description='Use the no_roof variant (recommended for training / top-down views).'
    ))
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_master_uri)
    ld.add_action(set_fastdds_builtin_transports)
    ld.add_action(gzserver_cmd_no_roof)
    ld.add_action(gzserver_cmd_roof)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_after_world)
    return ld
