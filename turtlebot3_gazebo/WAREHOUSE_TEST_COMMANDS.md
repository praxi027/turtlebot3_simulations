# Warehouse Test Commands

Run from `/home/ruslan/turtlebot3_ws`.

## Setup

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Rebuild

```bash
colcon build --packages-select turtlebot3_gazebo --symlink-install
```

## Launch v1

```bash
ros2 launch turtlebot3_gazebo nav2_aws_warehouse.launch.py \
  headless:=false use_rviz:=false gazebo_port:=11345
```

Example goal from `turtlebot3_gazebo/maps/warehouse/goals.yaml`:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.975, y: 6.625, z: 0.0}, orientation: {z: 0.839548, w: 0.543286}}}}"
```

## Launch v2

```bash
ros2 launch turtlebot3_gazebo nav2_aws_warehouse_v2.launch.py \
  headless:=false use_rviz:=false gazebo_port:=11401
```

Example goal from `turtlebot3_gazebo/maps/warehouse_v2/goals.yaml`:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.975, y: 6.625, z: 0.0}, orientation: {z: 0.705763, w: 0.708448}}}}"
```

## Launch v3

```bash
ros2 launch turtlebot3_gazebo nav2_aws_warehouse_v3.launch.py \
  headless:=false use_rviz:=false gazebo_port:=11402 spawn_pedestrians:=true
```

Example goal from `turtlebot3_gazebo/maps/warehouse_v3/goals.yaml`:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.975, y: 6.625, z: 0.0}, orientation: {z: 0.705763, w: 0.708448}}}}"
```

## Useful checks

Confirm localization:

```bash
ros2 topic echo /amcl_pose --once
```

Confirm the robot is publishing odom TF:

```bash
ros2 topic echo /odom --once
```

If one of the default Gazebo ports is already in use, pick another free
`gazebo_port:=<port>` value for that launch.
