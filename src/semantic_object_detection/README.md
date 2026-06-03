# Semantic Object Detection

ROS2 Humble perception package for the K12 Gazebo world.

The default pipeline is now RGB-D semantic segmentation instead of YOLO. It
publishes dense semantic images and navigation-friendly clouds/maps:

- `/semantic/label_image` (`mono8` class IDs)
- `/semantic/color_image` (colorized semantic labels)
- `/semantic/overlay_image` (RGB + semantic overlay)
- `/semantic/detections` (connected semantic components)
- `/semantic/points` (semantic `PointCloud2` with `rgb` and `label` fields)
- `/semantic/navigation_obstacles` (`PointCloud2` for Nav2 obstacle layers)
- `/semantic/local_costmap` (`OccupancyGrid` in `base_link`)
- `/semantic/classes` (JSON class legend)

## Run

```bash
cd ~/snav_ws
source install/setup.bash
ros2 launch semantic_object_detection object_detection.launch.py
```

or explicitly:

```bash
ros2 launch semantic_object_detection rgbd_semantic_segmentation.launch.py
```

Preview:

```bash
rqt_image_view /semantic/overlay_image
```

## Nav2

Use `/semantic/navigation_obstacles` as a `PointCloud2` observation source in
the Nav2 obstacle layer. A starter snippet is in:

```text
src/semantic_object_detection/config/nav2_semantic_obstacle_layer.yaml
```

## RTABMap

RTABMap should consume normal RGB-D topics and odometry only. Keep semantic
clouds as visualization/planning layers, not SLAM geometry input. A safe starter
parameter snippet is in:

```text
src/semantic_object_detection/config/rtabmap_semantic_rgbd.yaml
```

## Legacy YOLO

YOLO is disabled by default. To compare against it:

```bash
ros2 launch semantic_object_detection object_detection.launch.py enable_yolo:=true enable_combined:=true view_topic:=/detections/combined/image
```
