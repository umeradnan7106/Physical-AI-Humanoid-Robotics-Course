---
id: vslam-visual-slam
title: Visual SLAM
sidebar_position: 3
description: Implement cuVSLAM for visual odometry and localization without LiDAR.
keywords: [cuvslam, visual slam, stereo camera, isaac ros, pose estimation]
---

# Visual SLAM with cuVSLAM

**No LiDAR? No problem. Vision alone can map and localize.**

## What is VSLAM?

**Visual SLAM** (Simultaneous Localization and Mapping):
- Builds 3D map from camera images
- Tracks robot pose (position + orientation) in real-time
- No expensive LiDAR required

**cuVSLAM** (NVIDIA's GPU-accelerated implementation):
- Stereo camera input (left + right images)
- Runs at 30+ FPS on RTX GPUs
- Outputs pose as `nav_msgs/Odometry`
- Integrates with ROS 2 Nav2 for navigation

## Add Stereo Camera to Robot

### In Isaac Sim

**Create stereo rig**:
```python
from omni.isaac.sensor import Camera

# Left camera
left_cam = Camera(
    prim_path="/World/humanoid/head/camera_left",
    position=[0.05, 0.03, 0],
    frequency=30,
    resolution=(1280, 720)
)

# Right camera (baseline = 0.06m)
right_cam = Camera(
    prim_path="/World/humanoid/head/camera_right",
    position=[0.05, -0.03, 0],
    frequency=30,
    resolution=(1280, 720)
)
```

**Publish to ROS 2**:
```python
# Enable ROS 2 camera publishers (Action Graph)
import omni.graph.core as og

keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS_Cameras", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraLeft", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("CameraRight", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("CameraLeft.inputs:topicName", "/camera/left/image_raw"),
            ("CameraLeft.inputs:cameraPrim", "/World/humanoid/head/camera_left"),
            ("CameraRight.inputs:topicName", "/camera/right/image_raw"),
            ("CameraRight.inputs:cameraPrim", "/World/humanoid/head/camera_right"),
        ],
    },
)
```

## Install cuVSLAM

**Using Isaac ROS Docker** (recommended):
```bash
cd ~/workspaces/isaac_ros-dev
./scripts/run_dev.sh

# Inside container
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam
```

**Native install**:
```bash
sudo apt install ros-humble-isaac-ros-visual-slam
```

## Configure cuVSLAM

**camera_info.yaml** (calibration parameters):
```yaml
image_width: 1280
image_height: 720
camera_name: stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [700.0, 0.0, 640.0,
         0.0, 700.0, 360.0,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
```

## Launch cuVSLAM

**vslam.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'denoise_input_images': False,
                'rectified_images': True,
                'enable_debug_mode': False,
                'debug_dump_path': '/tmp/cuvslam',
                'enable_slam_visualization': True,
                'enable_observations_view': True,
                'enable_landmarks_view': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'camera_optical_frames': ['camera_left', 'camera_right']
            }],
            remappings=[
                ('stereo_camera/left/image', '/camera/left/image_raw'),
                ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                ('stereo_camera/right/image', '/camera/right/image_raw'),
                ('stereo_camera/right/camera_info', '/camera/right/camera_info')
            ]
        )
    ])
```

**Run**:
```bash
ros2 launch my_vslam vslam.launch.py
```

## Verify VSLAM

**Check topics**:
```bash
ros2 topic list
# Should show:
#   /visual_slam/tracking/odometry
#   /visual_slam/tracking/vo_pose
#   /visual_slam/status
```

**View pose**:
```bash
ros2 topic echo /visual_slam/tracking/odometry
```

**Output example**:
```
header:
  stamp: {sec: 10, nanosec: 500000000}
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position: {x: 1.23, y: -0.45, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.995}
```

## Visualize in RViz2

**vslam_rviz.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '$(find my_vslam)/rviz/vslam.rviz']
        )
    ])
```

**RViz config** (`vslam.rviz`):
- **Fixed Frame**: `map`
- Add display: **Odometry** → Topic: `/visual_slam/tracking/odometry`
- Add display: **PointCloud2** → Topic: `/visual_slam/vis/landmarks_cloud`
- Add display: **TF** → Show frames: `map`, `odom`, `base_link`

## Test VSLAM

**Move robot in Isaac Sim**:
```python
# Publish velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

**Expected behavior**:
- Blue trajectory in RViz shows robot path
- Point cloud shows mapped features
- TF tree updates: `map` → `odom` → `base_link`

## Troubleshooting

**"No pose output"**:
- Ensure environment has visual features (textured walls, furniture)
- Check camera calibration: `ros2 topic echo /camera/left/camera_info`
- Verify stereo baseline > 0.05m

**"Tracking lost"**:
- Add lighting to Isaac Sim scene
- Reduce robot velocity
- Enable image denoising: `denoise_input_images: True`

**"High CPU usage"**:
- Verify CUDA/GPU acceleration: `nvidia-smi` should show `isaac_ros_visual_slam` using GPU
- Reduce camera resolution to 640x480

## Compare: VSLAM vs LiDAR SLAM

| Feature | cuVSLAM | LiDAR SLAM (SLAM Toolbox) |
|---------|---------|---------------------------|
| **Sensor cost** | $100 (stereo camera) | $1,000+ (2D LiDAR) |
| **Range** | 5-10m | 30m+ |
| **Texture dependency** | Yes (fails in blank walls) | No (works in dark) |
| **3D mapping** | Yes (depth from stereo) | No (2D planar only) |
| **Compute** | GPU-intensive | CPU-moderate |

**Use cuVSLAM when**: Indoor navigation, cost-sensitive, GPU available

**Use LiDAR SLAM when**: Large outdoor spaces, low-texture environments

**Next**: [Nav2 Navigation Stack](/docs/module-03-isaac/nav2-navigation)
