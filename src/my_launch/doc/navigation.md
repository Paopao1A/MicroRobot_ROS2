# MicroRobot navigation notes

## Save a map

Start mapping first:

```bash
cd ~/MicroRos
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_launch mapping_bringup.launch.py
```

After driving around the environment, save the map:

```bash
~/MicroRos/src/my_launch/scripts/save_map.sh micro_robot_map
```

The default output directory is `~/MicroRos/maps/`.

## Start navigation

```bash
cd ~/MicroRos
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_launch navigation_bringup.launch.py
```

Use RViz `2D Pose Estimate` first, then send a short `Nav2 Goal`.

## Useful checks

```bash
ros2 topic echo /scan_filtered --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /local_costmap/published_footprint --once
```

If the robot gets too close to obstacles, tune `footprint` and `inflation_radius` in `src/my_launch/config/nav2_params.yaml`.
