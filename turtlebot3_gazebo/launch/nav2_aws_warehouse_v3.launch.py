#!/usr/bin/env python3
"""warehouse_v3 — v2 keepout layout plus three pedestrians.

Each pedestrian is spawned as a regular Gazebo Classic <model> (cylinder
visual + cylinder collision so the lidar sees it) driven along a closed loop
of waypoints by libpedestrian_plugin.so. Patrol paths trace the perimeter
corridors so the pedestrians genuinely block the medium/long-distance goals
without crowding the spawn cell.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (name, speed_m_s, [(x,y), ...]) — closed loop, last → first segment auto-added.
# Patrol lines must stay clear of both the static occupancy map and the v2/v3
# keepout zones; horizontal corridors above/below the S squares are blocked
# either by physical walls or by C5/C-zone keepouts, so v3 uses two vertical
# patrols in the central and spawn corridors. Validated by
# generate_warehouse_keepout.py — keep these in sync with WAREHOUSE_V3_PATROLS
# in that script.
PEDESTRIAN_PATROLS: list[tuple[str, float, list[tuple[float, float]]]] = [
    # Central corridor (between S squares and east shelves) — blocks the main
    # east-bound nav corridor. Starts north, walks south.
    ('ped_east', 0.6, [(1.5, 6.0), (1.5, -6.0)]),
    # Spawn corridor (between W0 wall and S squares) — passes the spawn cell
    # at offset 0.7 m so the robot routinely encounters it. Starts south.
    ('ped_west', 0.5, [(-3.7, -6.0), (-3.7, 6.0)]),
]

PED_RADIUS = 0.25
PED_HEIGHT = 1.7


def pedestrian_sdf(name: str, speed: float,
                   waypoints: list[tuple[float, float]]) -> str:
    points_xml = '\n        '.join(
        f'<point>{x:.3f} {y:.3f} 0</point>' for x, y in waypoints
    )
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="body">
      <pose>0 0 {PED_HEIGHT / 2:.3f} 0 0 0</pose>
      <inertial><mass>50.0</mass></inertial>
      <collision name="cyl">
        <geometry><cylinder><radius>{PED_RADIUS}</radius>
          <length>{PED_HEIGHT}</length></cylinder></geometry>
      </collision>
      <visual name="cyl">
        <geometry><cylinder><radius>{PED_RADIUS}</radius>
          <length>{PED_HEIGHT}</length></cylinder></geometry>
        <material><ambient>0.85 0.4 0.2 1</ambient>
          <diffuse>0.9 0.5 0.25 1</diffuse></material>
      </visual>
    </link>
    <plugin name="patrol" filename="libpedestrian_plugin.so">
      <speed>{speed}</speed>
      <z>0.0</z>
      <waypoints>
        {points_xml}
      </waypoints>
    </plugin>
  </model>
</sdf>'''


def spawn_pedestrian_node(name: str, speed: float,
                          waypoints: list[tuple[float, float]]) -> Node:
    sdf = pedestrian_sdf(name, speed, waypoints)
    # spawn_entity.py only accepts -file/-topic/-database/-stdin (no -string),
    # so write the SDF to a temp file alongside the launch process and pass
    # -file. Gets cleaned up by /tmp on reboot; cheap and avoids piping.
    fd, sdf_path = tempfile.mkstemp(prefix=f'{name}_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    x0, y0 = waypoints[0]
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{name}',
        output='screen',
        arguments=[
            '-entity', name,
            '-file', sdf_path,
            '-x', f'{x0:.3f}', '-y', f'{y0:.3f}', '-z', '0.0',
        ],
    )


def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    base_launch = os.path.join(
        pkg_tb3_gazebo, 'launch', 'nav2_aws_warehouse.launch.py'
    )
    # v3 reuses the v2 keepout layout — pedestrians are dynamic, so they live
    # outside the static keepout mask and Nav2 sees them only through the lidar.
    default_map = os.path.join(
        pkg_tb3_gazebo, 'maps', 'warehouse_v3', 'map.yaml'
    )
    default_keepout_mask = os.path.join(
        pkg_tb3_gazebo, 'maps', 'warehouse_v3', 'keepout_mask.yaml'
    )
    default_params = os.path.join(
        pkg_tb3_gazebo, 'params', 'nav2_aws_warehouse.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    nav2_params = LaunchConfiguration('nav2_params_file', default=default_params)
    keepout_mask = LaunchConfiguration('keepout_mask', default=default_keepout_mask)
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')
    gazebo_port = LaunchConfiguration('gazebo_port', default='11402')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    headless = LaunchConfiguration('headless', default='false')
    no_roof = LaunchConfiguration('no_roof', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    slam = LaunchConfiguration('slam', default='false')
    spawn_pedestrians = LaunchConfiguration('spawn_pedestrians', default='true')

    base_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'nav2_params_file': nav2_params,
            'keepout_mask': keepout_mask,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'gazebo_port': gazebo_port,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_z': initial_pose_z,
            'initial_pose_yaw': initial_pose_yaw,
            'headless': headless,
            'no_roof': no_roof,
            'use_rviz': use_rviz,
            'autostart': autostart,
            'slam': slam,
        }.items(),
    )

    # Defer spawning until /spawn_entity service is up — gzserver advertises it
    # only after world load, which lags the launch by ~5–8 s.
    spawn_actions = [
        spawn_pedestrian_node(name, speed, wpts)
        for name, speed, wpts in PEDESTRIAN_PATROLS
    ]
    spawn_after_world = TimerAction(
        period=8.0,
        actions=spawn_actions,
        condition=IfCondition(spawn_pedestrians),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('map', default_value=default_map))
    ld.add_action(DeclareLaunchArgument('nav2_params_file', default_value=default_params))
    ld.add_action(DeclareLaunchArgument('keepout_mask', default_value=default_keepout_mask))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-3.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('gazebo_port', default_value='11402'))
    ld.add_action(DeclareLaunchArgument('initial_pose_x', default_value=x_pose))
    ld.add_action(DeclareLaunchArgument('initial_pose_y', default_value=y_pose))
    ld.add_action(DeclareLaunchArgument('initial_pose_z', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('initial_pose_yaw', default_value=yaw))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('no_roof', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'spawn_pedestrians', default_value='true',
        description='Set false to launch v3 layout without dynamic pedestrians.'))
    ld.add_action(base_launch_cmd)
    ld.add_action(spawn_after_world)
    return ld
