# SNAV Workspace - Semantic Navigation System

**Last Updated:** May 24, 2026

## Project Overview

This ROS2 Humble workspace implements a comprehensive **semantic navigation system** for the K12 robot in a Gazebo simulation environment. The system integrates RGB-D camera processing, semantic object detection, point cloud generation, SLAM mapping, and navigation obstacle detection for autonomous navigation with semantic understanding.

---

## Workspace Structure

```
snav_ws/                           # Root ROS2 workspace
├── src/                           # Source packages
│   ├── k12_description/           # Robot URDF, models, and visualization configs
│   ├── semantic_navigation/       # Core semantic navigation package
│   └── semantic_object_detection/ # RGB-D semantic segmentation & detection
├── build/                         # Build artifacts (auto-generated)
├── install/                       # Installed packages (auto-generated)
├── log/                           # Build/test logs
├── datasets/                      # Training data and test datasets
│   └── k12_yolo/                 # YOLO training data (legacy)
└── .venv/                         # Python virtual environment
```

---

## Packages

### 1. **k12_description** (Robot Description)
**Type:** C++ (ament_cmake)  
**Purpose:** Defines the K12 robot's physical and visual properties

**Key Components:**
- **URDF Models:** Robot structure, links, and joints
- **Meshes:** 3D mesh files for robot visualization
- **Gazebo Worlds:** K12 world simulation environments
- **RViz Configuration:** Visualization presets

**Main Entry Points:**
- `view_robot.launch.py` - Launches robot visualization with optional semantic navigation
- Gazebo world launch files for simulation

---

### 2. **semantic_object_detection** (Perception Layer)
**Type:** Python (ament_python)  
**Version:** 0.0.1  
**Maintainer:** Rahiitya

**Purpose:** Processes RGB-D camera streams to produce semantic segmentation and object detection products
<img width="1920" height="1080" alt="Screenshot from 2026-06-03 15-59-15" src="https://github.com/user-attachments/assets/dc6578b0-ab3c-49be-b4b4-69e1b7d6ef8e" />


#### Key Nodes:

1. **rgbd_semantic_segmenter** (Main)
   - **File:** `semantic_object_detection/rgbd_semantic_segmenter.py`
   - **Functionality:**
     - Subscribes to RGB and depth images from `/camera/depth_camera/`
     - Performs RGB-D semantic segmentation (replaces legacy YOLO)
     - Generates 9 semantic classes: floor, wall, boxes (4 colors), object, wall_marker
     - Publishes dense semantic products for planning and visualization
   
   - **Published Topics:**
     - `/semantic/label_image` - Class ID image (mono8)
     - `/semantic/color_image` - Colorized semantic labels
     - `/semantic/overlay_image` - RGB + semantic overlay
     - `/semantic/detections` - Connected component detections
     - `/semantic/points` - Semantic PointCloud2 (RGB + label fields)
     - `/semantic/navigation_obstacles` - PointCloud2 for Nav2 obstacle layers
     - `/semantic/local_costmap` - OccupancyGrid in base_link frame
     - `/semantic/classes` - JSON legend of semantic classes

   - **Key Parameters:**
     - `process_every_n_frames`: Skip frames (default: 2)
     - `floor_start_ratio`: Floor detection threshold (default: 0.58)
     - `wall_stop_ratio`: Wall detection upper bound (default: 0.78)
     - `depth_edge_threshold_m`: Edge detection (default: 0.16m)
     - `point_decimation`: Reduce point cloud density (default: 4)
     - `costmap_resolution`: OccupancyGrid cell size (default: 0.05m)

   - **Semantic Classes:**
     | ID | Name | Color | Cost | Navigable |
     |----|------|-------|------|-----------|
     | 0 | unknown | (0,0,0) | -1 | No |
     | 1 | floor | (80,220,210) | 0 | Yes |
     | 2 | wall | (150,150,150) | 100 | No |
     | 3 | red_box | (0,0,255) | 100 | No |
     | 4 | blue_box | (255,0,0) | 100 | No |
     | 5 | green_box | (0,255,0) | 100 | No |
     | 6 | yellow_box | (0,255,255) | 100 | No |
     | 7 | purple_box | (255,0,255) | 100 | No |
     | 8 | object | (0,140,255) | 100 | No |
     | 9 | wall_marker | (255,255,0) | 80 | No |

2. **Legacy Detectors** (Disabled by Default)
   - `yolo_detector.py` - YOLO-based detection (legacy, DEPRECATED)
   - `color_detector.py` - Color-based segmentation (legacy)
   - `combined_detector.py` - Combines YOLO and color detection (legacy)

#### Launch Files:
- `object_detection.launch.py` - Main perception pipeline
  ```bash
  ros2 launch semantic_object_detection object_detection.launch.py
  ```
- `rgbd_semantic_segmentation.launch.py` - RGB-D segmentation pipeline
  ```bash
  ros2 launch semantic_object_detection rgbd_semantic_segmentation.launch.py
  ```

#### Configuration Files:
- `config/nav2_semantic_obstacle_layer.yaml` - Nav2 obstacle layer integration
- `config/rtabmap_semantic_rgbd.yaml` - RTAB-Map SLAM parameters

#### Dependencies:
- `rclpy`, `sensor_msgs`, `message_filters`, `cv_bridge`
- `opencv-python`, `numpy` - Image processing
- `vision_msgs`, `visualization_msgs` - Detection and visualization
- `gazebo_ros`, `rtabmap_slam` - Simulation and mapping

---

### 3. **semantic_navigation** (Navigation Layer)
**Type:** Python (ament_python)  
**Version:** 0.0.1  
**Maintainer:** Rahiitya

**Purpose:** Fuses RGB-D and semantic segmentation into navigation-ready point clouds and integrates SLAM

<img width="1920" height="1080" alt="Screenshot from 2026-06-03 16-02-08" src="https://github.com/user-attachments/assets/9913bf13-2161-4596-9d2d-0a128cc91d0a" />

#### Key Nodes:

1. **semantic_cloud_generator** (Main)
   - **File:** `semantic_navigation/semantic_cloud_generator.py` (477 lines)
   - **Functionality:**
     - Subscribes to RGB, depth, camera info, and semantic label images
     - Fuses them into semantic point clouds
     - Transforms points from camera frame to base_link, odom, and map frames
     - Accumulates points over time for persistent mapping
     - Applies voxel downsampling for memory efficiency
   
   - **Subscribed Topics:**
     - `/camera/depth_camera/image_raw` - RGB image
     - `/camera/depth_camera/depth/image_raw` - Depth image
     - `/camera/depth_camera/camera_info` - Camera intrinsics
     - `/semantic/label_image` - Semantic labels from segmenter
   
   - **Published Topics (Live):**
     - `/semantic/points` - Semantic cloud in base_link (6 Hz default)
     - `/semantic/navigation_obstacles` - Obstacle cloud in base_link for Nav2
   
   - **Published Topics (Persistent):**
     - `/semantic/map_points` - Accumulated semantic cloud in map frame (2 Hz default)
     - `/semantic/map_obstacles` - Accumulated obstacle cloud in map frame
   
   - **Key Parameters:**
     - `point_decimation`: Reduce density (default: 6)
     - `min_depth_m`: Min range (default: 0.12m)
     - `max_depth_m`: Max range (default: 6.0m)
     - `use_approx_sync`: Time synchronization mode (default: True)
     - `sync_slop_sec`: Sync tolerance (default: 0.05s)
     - `publish_rate_hz`: Live cloud rate (default: 6.0 Hz)
     - `publish_map_clouds`: Enable persistent mapping (default: True)
     - `map_publish_rate_hz`: Persistent cloud rate (default: 2.0 Hz)
     - `map_voxel_size_m`: Voxel size for downsampling (default: 0.10m)
     - `max_map_points`: Limit persistent cloud size (default: 100,000 points)
     - `obstacle_label_ids`: Which classes are obstacles (default: 2,3,4,5,6,7,8,9)
<img width="1920" height="1080" alt="Screenshot from 2026-06-03 15-59-35" src="https://github.com/user-attachments/assets/347fb0dd-0738-4f84-b322-dfe01ba0ab43" />


   - **Architecture Highlights:**
     - Uses TF2 to transform points between frames
     - Synchronized subscription to RGB-D and semantic inputs
     - Voxel-based map accumulation for memory efficiency
     - Colorizes points by semantic label or RGB value

   - **Important Design Rule:**
     > Semantic clouds are visualization and planning layers ONLY.
     > Do NOT remap `/semantic/navigation_obstacles` into RTAB-Map's scan/scan_cloud.

2. **gazebo_semantic_landmarks.py**
   - Manages semantic landmarks detected in Gazebo simulation

3. **semantic_goal_resolver.py**
   - Resolves semantic goals (e.g., "go to the red box") into map coordinates
<img width="1920" height="1080" alt="Screenshot from 2026-06-03 15-59-50" src="https://github.com/user-attachments/assets/9ac03338-2748-474f-8ebe-5fc34375398a" />


#### Launch Files:
- `semantic_navigation.launch.py` - Core semantic navigation system
  ```bash
  ros2 launch semantic_navigation semantic_navigation.launch.py
  ```
  Launches:
  - `semantic_object_detection/rgbd_semantic_segmenter`
  - `semantic_navigation/semantic_cloud_generator`
  - RTAB-Map RGB-D SLAM using RGB, depth, and odometry only

- `k12_rgbd_mapping.launch.py` - Full system integration
  ```bash
  ros2 launch semantic_navigation k12_rgbd_mapping.launch.py
  ```
  Launches:
  - Gazebo simulator
  - K12 robot in K12 world
  - Semantic perception pipeline
  - RTAB-Map SLAM
  - RViz2 visualization
  - rtabmapviz for SLAM debugging

#### Configuration Files:
- `config/nav2_semantic_costmap.yaml` - Nav2 costmap integration
- `config/rtabmap_rgbd_semantic.yaml` - RTAB-Map parameters

#### Data Flow (Topic Pipeline):
```
[Gazebo/Camera]
    ↓
    ├─→ RGB image ──┐
    └─→ Depth + CameraInfo
           ↓
    [semantic_object_detection/rgbd_semantic_segmenter]
           ↓
         Semantic labels
           ↓
    [semantic_navigation/semantic_cloud_generator]
           ↓
    ├─→ /semantic/points (live)
    ├─→ /semantic/navigation_obstacles
    ├─→ /semantic/map_points (persistent)
    └─→ /semantic/map_obstacles (persistent)
           ↓
    [Nav2 + RViz Visualization]
```

#### Dependencies:
- All from semantic_object_detection
- `tf2_ros` - Transform management
- `rtabmap_slam`, `rtabmap_viz` - SLAM and visualization
- `rviz2` - 3D visualization

---

## Topic Architecture Summary

### Camera & Sensor Inputs
| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/camera/depth_camera/image_raw` | Image | Gazebo | RGB camera stream |
| `/camera/depth_camera/depth/image_raw` | Image | Gazebo | Depth camera stream |
| `/camera/depth_camera/camera_info` | CameraInfo | Gazebo | Camera intrinsics |
| `/odom` | Odometry | Gazebo | Robot odometry |

### Semantic Perception
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/semantic/label_image` | Image (mono8) | rgbd_semantic_segmenter | Semantic class IDs |
| `/semantic/color_image` | Image (BGR8) | rgbd_semantic_segmenter | Colorized labels |
| `/semantic/overlay_image` | Image (BGR8) | rgbd_semantic_segmenter | RGB + label overlay |
| `/semantic/detections` | Detection2DArray | rgbd_semantic_segmenter | Connected components |
| `/semantic/classes` | String (JSON) | rgbd_semantic_segmenter | Class legend |

### Semantic Point Clouds
| Topic | Type | Frame | Publisher | Rate | Purpose |
|-------|------|-------|-----------|------|---------|
| `/semantic/points` | PointCloud2 | base_link | semantic_cloud_generator | 6 Hz | Live semantic cloud |
| `/semantic/navigation_obstacles` | PointCloud2 | base_link | semantic_cloud_generator | 6 Hz | Nav2 obstacle input |
| `/semantic/map_points` | PointCloud2 | map | semantic_cloud_generator | 2 Hz | Persistent semantic map |
| `/semantic/map_obstacles` | PointCloud2 | map | semantic_cloud_generator | 2 Hz | Persistent obstacle map |

### SLAM & Mapping
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/map` | OccupancyGrid | RTAB-Map | Occupancy map from SLAM |
| `/rtabmap/octomap_grid` | OccupancyGrid | RTAB-Map | 3D occupancy (if enabled) |
| `/tf` | TFMessage | Robot + SLAM | Transform tree |

### Local Costmap (if enabled)
| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/semantic/local_costmap` | OccupancyGrid | rgbd_semantic_segmenter | Local obstacle grid in base_link |

---

## Frame Transforms

```
map (fixed frame in RViz)
  ↓ [map → odom] (published by RTAB-Map SLAM)
odom (odometry frame)
  ↓ [odom → base_link] (published by Gazebo)
base_link (robot center)
  ↓ [base_link → camera_depth_optical_frame] (from URDF)
camera_depth_optical_frame (camera)
```

**Key Points:**
- RViz uses `map` as its fixed frame
- Gazebo publishes `odom → base_link` odometry
- RTAB-Map publishes `map → odom` for localization
- Camera offset from base_link: (0.59, 0.0, 0.32) meters by default
- Semantic clouds are published in `base_link` and `map` frames

---

## Build & Setup

### Prerequisites
- ROS2 Humble installed
- Gazebo installed
- Python 3.10+
- OpenCV, NumPy installed

### Build the Workspace
```bash
cd ~/snav_ws
colcon build --symlink-install
```

### Source the Workspace
```bash
source install/setup.bash
```

---

## Running the System

### Option 1: Semantic Navigation Only (No Gazebo)
Requires external RGB-D camera topics:
```bash
ros2 launch semantic_navigation semantic_navigation.launch.py
```

### Option 2: Full Simulation (Recommended)
Complete system with Gazebo, SLAM, and visualization:
```bash
ros2 launch semantic_navigation k12_rgbd_mapping.launch.py
```

### Option 3: Perception Only
Just the semantic segmentation pipeline:
```bash
ros2 launch semantic_object_detection rgbd_semantic_segmentation.launch.py
```

### Visualization
After launching the system, view the semantic outputs:

**RViz2 (Recommended):**
```bash
ros2 launch k12_description view_robot.launch.py launch_semantic_navigation:=true
```

**Individual Streams:**
```bash
# View semantic color image
rqt_image_view /semantic/color_image

# View semantic overlay
rqt_image_view /semantic/overlay_image

# View label image
rqt_image_view /semantic/label_image
```

**RTAB-Map Visualizer:**
```bash
rtabmapviz
```
(Launches as part of k12_rgbd_mapping.launch.py)

---

## Integration with Nav2

To use semantic clouds with Nav2 navigation:

1. **Add Semantic Obstacle Layer** to Nav2's local and global costmaps
2. Use configuration from: `src/semantic_object_detection/config/nav2_semantic_obstacle_layer.yaml`
3. Configure as PointCloud2 observation sources pointing to:
   - Local: `/semantic/navigation_obstacles`
   - Global: `/semantic/map_obstacles`

Example integration in nav2_params.yaml:
```yaml
local_costmap:
  costmap:
    plugins: ["obstacle_layer", "semantic_layer", "inflation_layer"]
    semantic_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "semantic_obs"
      semantic_obs: {topic: "/semantic/navigation_obstacles", sensor_frame: "base_link"}

global_costmap:
  costmap:
    plugins: ["static_layer", "semantic_layer", "inflation_layer"]
    semantic_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "semantic_map_obs"
      semantic_map_obs: {topic: "/semantic/map_obstacles", sensor_frame: "map"}
```

---

## Integration with SLAM

**RTAB-Map Configuration:**
- Uses RGB and depth images as primary SLAM inputs
- Odometry from Gazebo `/odom` for loop closure
- **Semantic clouds are NOT fed to SLAM** (kept as visualization/planning layers only)
- Configuration: `src/semantic_navigation/config/rtabmap_rgbd_semantic.yaml`
- SLAM output: `/map` OccupancyGrid in map frame

### Map Accuracy Baseline (K12 Gazebo)

For metrically stable maps (no wall spreading/ghosting), keep these defaults:
- `map -> odom -> base_link` TF chain only (no extra `map->odom` publisher)
- RTAB-Map RGB-D sync: `approx_sync=false`, `queue_size=20`
- Keyframe updates: `RGBD/LinearUpdate=0.08`, `RGBD/AngularUpdate=0.05`
- Grid filtering: `Grid/CellSize=0.05`, `Grid/RangeMin=0.30`, `Grid/RangeMax=6.0`
- Cloud filtering: `Cloud/Decimation=4`, `Cloud/VoxelSize=0.07`
- Semantic clouds only for planning/visualization, never `scan`/`scan_cloud` input to RTAB-Map

Useful verification commands:
```bash
ros2 topic hz /camera/depth_camera/image_raw
ros2 topic hz /camera/depth_camera/depth/image_raw
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

---

## Development Progress

### Completed Features ✅

#### Phase 1: Infrastructure
- [x] ROS2 Humble workspace setup (colcon)
- [x] Package structure: semantic_navigation + semantic_object_detection + k12_description
- [x] Launch system framework
- [x] Topic naming conventions

#### Phase 2: Perception (Semantic Segmentation)
- [x] RGB-D semantic segmentation node (`rgbd_semantic_segmenter.py`)
- [x] 9-class semantic taxonomy (floor, wall, boxes, object, marker)
- [x] Dense semantic label image output
- [x] Colorized semantic visualization
- [x] Semantic-to-PointCloud conversion
- [x] Connected component detection (regions/clusters)
- [x] Navigation obstacle extraction from semantic classes
- [x] Local occupancy grid generation from semantic labels

#### Phase 3: Point Cloud Fusion
- [x] Semantic cloud generator (`semantic_cloud_generator.py`)
- [x] RGB-D fusion with semantic labels
- [x] Camera-to-base_link transformation
- [x] Synchronized subscription using message_filters
- [x] Point cloud downsampling (decimation & voxel grid)
- [x] Range filtering (min/max depth)
- [x] Live point cloud publishing (base_link frame)
- [x] Persistent map accumulation
- [x] Spatial downsampling for memory efficiency
- [x] Dual cloud publishing (semantic + navigation obstacles)

#### Phase 4: SLAM Integration
- [x] RTAB-Map RGB-D SLAM integration
- [x] Odometry pipeline setup
- [x] Transform tree management (map → odom → base_link)
- [x] Map frame semantic cloud publishing

#### Phase 5: Visualization
- [x] RViz2 integration with semantic overlays
- [x] Multi-topic visualization (RGB, depth, segmentation)
- [x] Point cloud visualization (live + persistent)
- [x] Obstacle cloud visualization for Nav2 preview
- [x] rtabmapviz integration for SLAM debugging

#### Phase 6: Navigation Integration
- [x] Semantic obstacle cloud publishing for Nav2
- [x] Nav2 costmap layer configuration example
- [x] Local costmap integration (base_link frame)

#### Phase 7: Parameter Management
- [x] Comprehensive parameter declarations
- [x] YAML configuration files for common settings
- [x] Launch-time parameter overrides
- [x] Default parameter tuning for K12 robot

#### Phase 8: Documentation
- [x] Per-package README.md files
- [x] Topic and frame documentation
- [x] Class taxonomy documentation
- [x] Parameter reference guides

### Current State
- **Build Status:** Successful (all 3 packages compile)
- **Runtime Status:** Tested with Gazebo K12 simulation
- **Test Coverage:** Integration tests via real-world launching

### Known Issues / Limitations
- Legacy YOLO detector disabled (kept for reference)
- Color detection deprecated (replaced by RGB-D semantic segmentation)
- Memory consumption scales with map size (managed by voxel downsampling)
- No active loop closure on small/featureless floors (handled by RTAB-Map tuning)

### Future Enhancements
- [ ] Dynamic semantic class remapping
- [ ] Real-world camera calibration workflow
- [ ] Depth completion for sparse regions
- [ ] Semantic panoptic segmentation (instance tracking)
- [ ] Temporal consistency filtering for flickering labels
- [ ] GPU acceleration for segmentation (if available)
- [ ] Multi-resolution costmaps
- [ ] Trajectory prediction using semantic context
- [ ] Semantic goal planning ("pick up the blue box")
- [ ] Integration with manipulation pipeline

---

## File Structure Detail

### Package: semantic_object_detection

```
semantic_object_detection/
├── semantic_object_detection/
│   ├── __init__.py
│   ├── rgbd_semantic_segmenter.py    # Main segmentation node (523 lines)
│   ├── yolo_detector.py              # Legacy YOLO (deprecated)
│   ├── color_detector.py             # Legacy color-based (deprecated)
│   └── combined_detector.py          # Legacy combination (deprecated)
├── launch/
│   ├── object_detection.launch.py    # Main launch file
│   └── rgbd_semantic_segmentation.launch.py
├── config/
│   ├── nav2_semantic_obstacle_layer.yaml
│   └── rtabmap_semantic_rgbd.yaml
├── resource/
│   └── semantic_object_detection
├── package.xml
├── setup.py
└── setup.cfg
```

### Package: semantic_navigation

```
semantic_navigation/
├── semantic_navigation/
│   ├── __init__.py
│   ├── semantic_cloud_generator.py   # Main cloud fusion (477 lines)
│   ├── gazebo_semantic_landmarks.py
│   └── semantic_goal_resolver.py
├── launch/
│   ├── semantic_navigation.launch.py # Core system launch
│   └── k12_rgbd_mapping.launch.py    # Full integration with Gazebo
├── config/
│   ├── nav2_semantic_costmap.yaml
│   └── rtabmap_rgbd_semantic.yaml
├── resource/
│   └── semantic_navigation
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
└── ...
```

### Package: k12_description

```
k12_description/
├── urdf/
│   └── k12.urdf.xacro (K12 robot definition)
├── meshes/
│   └── (3D models for robot parts)
├── models/
│   └── (Gazebo models)
├── worlds/
│   └── k12_world.sdf (Gazebo environment)
├── launch/
│   ├── view_robot.launch.py
│   └── (other launch files)
├── rviz/
│   └── (RViz configuration files)
├── src/
│   └── (C++ source files if any)
├── include/
│   └── (C++ headers)
├── CMakeLists.txt
├── package.xml
└── ...
```

---

## Key Design Decisions

1. **Semantic Over Detection:** Switched from YOLO bounding boxes to dense semantic segmentation for better coverage and memory efficiency in SLAM.

2. **Separate Clouds:** Maintains both semantic points (visualization) and obstacle points (planning) to avoid coupling semantic labels with navigation geometry.

3. **Frame Hierarchy:** Publishes clouds in both `base_link` (live, local) and `map` (persistent, global) frames for flexible use in navigation.

4. **Voxel Accumulation:** Uses voxel-based downsampling for persistent maps to keep memory usage bounded while maintaining spatial coverage.

5. **Message Filter Sync:** Uses `ApproximateTimeSynchronizer` to handle realistic timing mismatches between camera streams and semantic processing.

6. **Parameter-Driven:** All critical thresholds exposed as parameters for runtime tuning without recompilation.

7. **SLAM-Semantic Separation:** Semantic clouds are kept separate from SLAM geometry to prevent semantic label noise from affecting localization.

---

## Performance Notes

- **Perception Rate:** 6 Hz for live semantic clouds (limited by semantic segmentation speed)
- **Map Rate:** 2 Hz for persistent mapping (limited by accumulation efficiency)
- **Memory:** ~100k points max in persistent clouds (configurable)
- **CPU:** Single-threaded, runs on mid-range laptops with GPU acceleration optional

---

## RTAB-Map Mapping Issues (FIXED May 24, 2026)

**CRITICAL FIXES APPLIED:**
- ✅ Changed synchronization from exact to approximate (prevents frame drops)
- ✅ Enabled odometry information subscription (better pose estimation)
- ✅ Increased feature detection (1000 keypoints, 3x3 grid)
- ✅ Improved loop closure parameters (3 neighbors, 50-deep search)
- ✅ Fixed motion thresholds (0.02 rad, 0.05m for finer tracking)
- ✅ Enabled full graph optimization on loop closure
- ✅ Reduced point decimation (use 2x instead of 4x sampling)
- ✅ Made map updates continuous

**See [RTABMAP_TUNING_GUIDE.md](RTABMAP_TUNING_GUIDE.md) for:**
- Detailed explanation of 11 critical issues fixed
- Parameter-by-parameter tuning guide
- Debugging checklist
- Expected performance metrics
- Advanced database management

---

## Troubleshooting

### No semantic labels published
1. Check RGB-D input topics: `ros2 topic list | grep camera`
2. Verify segmenter is running: `ros2 node list | grep segmenter`
3. Check parameters: `ros2 param list /rgbd_semantic_segmenter`

### Points not transformed correctly
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify camera calibration in launch file
3. Check camera_info topic: `ros2 topic echo /camera/depth_camera/camera_info`

### Low point cloud density
1. Increase decimation parameter (default: 6, range: 1-10)
2. Reduce min_depth (but watch for noise)
3. Increase max_depth if environment is larger

### RViz not showing clouds
1. Change fixed frame to `map` or `base_link`
2. Add PointCloud2 displays for `/semantic/points`
3. Verify topic names in RViz config

### RTAB-Map Inaccurate Mapping (SOLVED)
**Root causes identified and fixed:**
1. Exact time sync too strict → Changed to approximate with 30-frame buffer
2. Odometry not subscribed → Now uses odometry info for better estimates
3. Too few features (700) → Increased to 1000 with better spatial distribution
4. Loop closure not working → Increased graph search depth (30→50) and neighbors (1→3)
5. Motion thresholds too loose → Reduced for finer detail tracking
6. See **[RTABMAP_TUNING_GUIDE.md](RTABMAP_TUNING_GUIDE.md)** for complete debugging procedure

---

## References

- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **RTAB-Map:** http://introlab.github.io/rtabmap/
- **Nav2:** https://navigation.ros.org/
- **Gazebo Classic:** http://gazebosim.org/

---

## Contact & Contribution

- **Maintainer:** Rahiitya
- **License:** Apache-2.0
- **Last Build:** Multiple successful builds logged in `/log/` directory
- **Latest Build:** `build_2026-05-20_09-24-52/`

---

## Quick Commands Reference

```bash
# Build
cd ~/snav_ws && colcon build --symlink-install

# Source
source install/setup.bash

# Full system (Gazebo + SLAM + RViz)
ros2 launch semantic_navigation k12_rgbd_mapping.launch.py

# Perception only
ros2 launch semantic_object_detection rgbd_semantic_segmentation.launch.py

# Core navigation (expects RGB-D input)
ros2 launch semantic_navigation semantic_navigation.launch.py

# Visualize robot
ros2 launch k12_description view_robot.launch.py launch_semantic_navigation:=true

# List active topics
ros2 topic list

# Echo a topic
ros2 topic echo /semantic/label_image

# View RViz
rviz2 -d /path/to/config.rviz

# View topic in rqt
rqt_image_view /semantic/overlay_image
```

---

**End of README**
