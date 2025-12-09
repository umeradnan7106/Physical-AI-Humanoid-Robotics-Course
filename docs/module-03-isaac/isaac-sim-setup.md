---
id: isaac-sim-setup
title: Isaac Sim Setup
sidebar_position: 2
description: Install NVIDIA Isaac Sim and configure for ROS 2 integration.
keywords: [isaac sim installation, omniverse launcher, gpu requirements, isaac ros]
---

# Isaac Sim Setup

## System Requirements

**Minimum**:
- NVIDIA RTX 2070 (8GB VRAM)
- Ubuntu 22.04
- 32GB RAM
- 50GB free storage

**Recommended**:
- NVIDIA RTX 4080/A6000 (16GB+ VRAM)
- 64GB RAM
- NVMe SSD

**Check GPU drivers**:
```bash
nvidia-smi
# Should show driver version 525+ and CUDA 12+
```

## Installation Steps

### 1. Install Omniverse Launcher

Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/):

```bash
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

### 2. Install Isaac Sim via Launcher

1. **Open Launcher** → Exchange tab
2. **Search**: "Isaac Sim"
3. **Install**: Isaac Sim 2023.1.1 or later
4. **Wait**: ~20GB download + install time

**Alternative (Docker)**:
```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --gpus all -it -v ~/isaac-data:/root/Documents \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### 3. First Launch

**From Launcher**: Click "LAUNCH" under Isaac Sim

**From terminal**:
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

**Headless mode** (servers):
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh --/app/window/width=0
```

## Interface Overview

**Main viewport**: 3D scene (RTX rendering)
**Content browser**: Asset library (bottom panel)
**Stage panel**: Scene hierarchy (right)
**Property panel**: Selected object settings (right)

**Essential shortcuts**:
- **F**: Frame selected object
- **Space**: Play/pause simulation
- **Ctrl+S**: Save scene
- **Shift+S**: Take screenshot

## Import Humanoid Robot

### Method 1: URDF Importer

```python
# In Isaac Sim Python console (Window → Script Editor)
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False

urdf_interface.parse_urdf(
    "/path/to/humanoid.urdf",
    "/World/humanoid",
    import_config
)
```

### Method 2: USD Export from Blender

1. **Export from Blender**: File → Export → USD (.usd)
2. **Import in Isaac**: File → Open → Select .usd file
3. **Add collision**: Right-click mesh → Add → Physics → Collision

## Configure ROS 2 Bridge

### Install Isaac ROS

**Option A: Docker (recommended)**:
```bash
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
./scripts/run_dev.sh
```

**Option B: Native**:
```bash
sudo apt install ros-humble-isaac-ros-base
```

### Enable ROS Bridge in Isaac Sim

```python
# In Isaac Sim Script Editor
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.graph.core as og

# Create ROS 2 clock publisher
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS_Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
        ],
    },
)
```

**Verify**:
```bash
# In terminal
ros2 topic list
# Should show /clock
ros2 topic echo /clock
```

## Load Example Scene

**Pre-built environments**:
- `Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd`
- `Isaac/Environments/Hospital/hospital.usd`
- `Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd`

**Load**:
1. Content browser → Isaac → Environments
2. Double-click `warehouse_multiple_shelves.usd`
3. Press **Space** to start simulation

## Common Issues

**"Isaac Sim crashes on launch"**:
```bash
# Check Vulkan support
vulkaninfo | grep deviceName

# Update drivers
sudo ubuntu-drivers autoinstall
sudo reboot
```

**"ROS 2 topics not appearing"**:
- Check `ROS_DOMAIN_ID` matches between Isaac Sim and host
- Restart Isaac Sim after changing environment variables
- Verify bridge extension enabled: Window → Extensions → search "ros2_bridge"

**"Poor FPS"**:
- Reduce viewport resolution: Window → Viewport → Resize
- Disable RTX: Edit → Preferences → Rendering → Uncheck "Enable RTX"
- Use Fast mode: Rendering → Render Settings → Mode → Fast

## Test Installation

**Spawn cube test**:
```python
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core import World

world = World()
cube = DynamicCuboid(prim_path="/World/Cube", position=[0, 0, 1.0], size=0.5)
world.reset()
```

Press **Space**—cube should fall due to gravity.

## Performance Tuning

**For training (speed over quality)**:
```python
import carb
settings = carb.settings.get_settings()
settings.set("/rtx/raytracing/subsurface/enabled", False)
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/translucency/enabled", False)
```

**For realistic rendering (quality over speed)**:
```python
settings.set("/rtx/pathtracing/spp", 16)  # Samples per pixel
settings.set("/rtx/pathtracing/totalSpp", 64)
settings.set("/rtx/post/aa/op", 3)  # TAA antialiasing
```

**Next**: [Visual SLAM (cuVSLAM)](/docs/module-03-isaac/vslam-visual-slam)
