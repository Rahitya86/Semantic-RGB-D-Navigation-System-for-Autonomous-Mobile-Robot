# semantic_navigation

ROS2 Humble RGB-D semantic navigation package for the K12 Gazebo world.

It launches:

- `semantic_object_detection/rgbd_semantic_segmenter` to publish `/semantic/label_image`
- `semantic_cloud_generator` to fuse RGB, depth, camera info, and semantic labels into `/semantic/points`
- `cmd_vel_limiter` to smooth and cap teleop/Nav2 commands (`/cmd_vel -> /cmd_vel_smoothed`)
- RTAB-Map RGB-D SLAM using RGB, depth, and `/odom` only

Main topics:

- Input RGB: `/camera/depth_camera/image_raw`
- Input depth: `/camera/depth_camera/depth/image_raw`
- Input semantic labels: `/semantic/label_image`
- Live semantic cloud in `base_link`: `/semantic/points`
- Live obstacle cloud in `base_link` for Nav2/collision visualization: `/semantic/navigation_obstacles`
- Persistent semantic cloud in `map`: `/semantic/map_points`
- Persistent obstacle cloud in `map` for visualization/semantic memory: `/semantic/map_obstacles`
- RTAB-Map map: `/map`

Run:

```bash
source install/setup.bash
ros2 launch semantic_navigation semantic_navigation.launch.py
```

Full Gazebo + RGB-D mapping + RViz2 + rtabmapviz:

```bash
ros2 launch semantic_navigation k12_rgbd_mapping.launch.py
```

Visualize with the K12 RViz launch:

```bash
ros2 launch k12_description view_robot.launch.py launch_semantic_navigation:=true
```

RViz uses `map` as its fixed frame. Gazebo publishes `odom -> base_link`, and
RTAB-Map publishes `map -> odom`, so the map stays fixed while the robot model
moves through it.

For Nav2, merge `config/nav2_semantic_costmap.yaml` into the robot Nav2 params so:
- local costmap observes `/semantic/navigation_obstacles`
- global costmap also observes `/semantic/navigation_obstacles` so clearing raytraces
  from the robot/sensor frame instead of from a persistent map-frame cloud.

Important architecture rule: semantic clouds are visualization and planning layers only.
Do not remap `/semantic/navigation_obstacles` into RTAB-Map `scan` or `scan_cloud`.

To reuse a previously built map without adding duplicate nodes, start localization mode:

```bash
ros2 launch semantic_navigation semantic_navigation.launch.py \
  rtabmap_incremental_memory:=false \
  rtabmap_init_wm_with_all_nodes:=true
```
