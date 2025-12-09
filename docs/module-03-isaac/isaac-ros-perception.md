---
id: isaac-ros-perception
title: Isaac ROS Perception
sidebar_position: 5
description: GPU-accelerated perception with Isaac ROS GEMs (DNN inference, AprilTags, pose estimation).
keywords: [isaac ros gems, dnn inference, apriltag, pose estimation, gpu acceleration]
---

# Isaac ROS Perception

**CPU perception: 5 FPS. GPU perception: 60 FPS.**

## Isaac ROS GEMs

**GEMs** (GPU-Enabled Modules) are ROS 2 nodes with NVIDIA GPU acceleration:
- **10-100× faster** than CPU-based equivalents
- TensorRT-optimized deep learning inference
- CUDA kernels for image processing

**Available GEMs**:
- **Visual SLAM**: cuVSLAM (covered earlier)
- **Object Detection**: DOPE, Detectnet
- **Pose Estimation**: FoundationPose, CenterPose
- **Segmentation**: U-Net, ESS (stereo depth)
- **AprilTag**: Fiducial marker detection
- **Image Processing**: Resize, rectify, encode

## Install Isaac ROS Perception

**Docker container** (recommended):
```bash
cd ~/workspaces/isaac_ros-dev
./scripts/run_dev.sh

# Inside container
sudo apt update
sudo apt install ros-humble-isaac-ros-apriltag \
                 ros-humble-isaac-ros-dnn-inference \
                 ros-humble-isaac-ros-image-proc
```

## AprilTag Detection

**What are AprilTags?** Black/white fiducial markers for pose estimation (like QR codes but optimized for robotics).

**Use cases**: Robot localization, object tracking, calibration targets

### Setup AprilTag Detection

**apriltag.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_apriltag',
            executable='isaac_ros_apriltag',
            parameters=[{
                'size': 0.16,  # Tag size in meters
                'max_tags': 64,
                'tag_family': 'tag36h11'
            }],
            remappings=[
                ('image', '/camera/left/image_raw'),
                ('camera_info', '/camera/left/camera_info')
            ]
        )
    ])
```

**Run**:
```bash
ros2 launch my_perception apriltag.launch.py
```

**Output topics**:
```bash
/tag_detections  # AprilTagDetectionArray (IDs + 3D poses)
/tag_poses       # PoseArray (visualization)
```

**Visualize in RViz**:
- Add **PoseArray** display → Topic: `/tag_poses`
- Axes show tag orientation (X=red, Y=green, Z=blue)

### Print AprilTags

Download from [AprilTag Generator](https://github.com/AprilRobotics/apriltag-imgs):
```bash
wget https://github.com/AprilRobotics/apriltag-imgs/raw/master/tag36h11/tag36_11_00000.png
# Print at 16cm × 16cm
```

## DNN Inference with TensorRT

**Example: Object detection with Detectnet (NVIDIA PeopleNet model)**

### Download Pre-trained Model

```bash
# Inside Isaac ROS container
cd /workspaces/isaac_ros-dev
ros2 run isaac_ros_dnn_inference download_models.sh --model-type detectnet
```

### Launch Detectnet

**detectnet.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_dnn_inference',
            executable='isaac_ros_dnn_inference',
            parameters=[{
                'model_file_path': '/tmp/models/detectnet/detectnet.onnx',
                'engine_file_path': '/tmp/models/detectnet/detectnet.plan',
                'input_tensor_names': ['input_tensor'],
                'output_tensor_names': ['output_cov', 'output_bbox'],
                'input_binding_names': ['input_1:0'],
                'output_binding_names': ['output_cov/Sigmoid:0', 'output_bbox/BiasAdd:0']
            }],
            remappings=[
                ('tensor_pub', '/inference/tensors'),
                ('image', '/camera/left/image_raw')
            ]
        ),
        Node(
            package='isaac_ros_detectnet',
            executable='isaac_ros_detectnet_visualizer',
            remappings=[
                ('detections', '/detections'),
                ('image', '/camera/left/image_raw')
            ]
        )
    ])
```

**Run**:
```bash
ros2 launch my_perception detectnet.launch.py
```

**View detections**:
```bash
ros2 topic echo /detections
# Shows bounding boxes for detected people/objects
```

## Depth Estimation with ESS

**ESS** (Efficient Semi-Global Stereo) computes depth from stereo images on GPU.

**ess_stereo.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_ess',
            executable='isaac_ros_ess',
            parameters=[{
                'engine_file_path': '/tmp/models/ess/ess.engine',
                'threshold': 0.9
            }],
            remappings=[
                ('left/image_rect', '/camera/left/image_raw'),
                ('right/image_rect', '/camera/right/image_raw'),
                ('left/camera_info', '/camera/left/camera_info'),
                ('right/camera_info', '/camera/right/camera_info')
            ]
        )
    ])
```

**Output**: `/disparity` (DisparityImage) → convert to depth:
```
depth (meters) = (focal_length × baseline) / disparity
```

## Performance Comparison

**Object detection benchmark** (640×480 @ 30 FPS):

| Implementation | Latency | GPU Usage |
|----------------|---------|-----------|
| **CPU (OpenCV DNN)** | 200ms (5 FPS) | N/A |
| **GPU (PyTorch)** | 50ms (20 FPS) | 60% |
| **TensorRT (Isaac ROS)** | 8ms (120 FPS) | 25% |

**Key insight**: Isaac ROS achieves 15× speedup over PyTorch by using INT8 precision, kernel fusion, and CUDA graphs.

## Custom DNN Models

**Convert PyTorch → ONNX → TensorRT**:

```python
import torch
import torch.onnx

# 1. Export PyTorch model to ONNX
model = YourPyTorchModel()
dummy_input = torch.randn(1, 3, 640, 480)
torch.onnx.export(model, dummy_input, "model.onnx")
```

```bash
# 2. Convert ONNX to TensorRT engine
/usr/src/tensorrt/bin/trtexec \
  --onnx=model.onnx \
  --saveEngine=model.engine \
  --fp16  # Enable half-precision
```

```python
# 3. Load in Isaac ROS DNN Inference node
# (Use detectnet.launch.py pattern, update paths)
```

## Sensor Fusion: Combine VSLAM + AprilTags

**Use case**: VSLAM for continuous localization, AprilTags for drift correction.

**Fusion strategy**:
1. cuVSLAM provides odometry at 30 Hz
2. AprilTag detection triggers pose correction when tag visible
3. robot_localization EKF fuses both sources

**ekf.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            parameters=[{
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'odom0': '/visual_slam/tracking/odometry',
                'odom0_config': [True, True, True,  # x, y, z
                                 True, True, True,  # roll, pitch, yaw
                                 False, False, False,  # vx, vy, vz
                                 False, False, False,  # vroll, vpitch, vyaw
                                 False, False, False],  # ax, ay, az
                'pose0': '/tag_detections',
                'pose0_config': [True, True, True, True, True, True,
                                 False, False, False, False, False, False,
                                 False, False, False]
            }]
        )
    ])
```

**Result**: Fused odometry on `/odometry/filtered` (more accurate than VSLAM alone).

**Next**: [Warehouse Navigation Project](/docs/module-03-isaac/project-isaac-navigation)
