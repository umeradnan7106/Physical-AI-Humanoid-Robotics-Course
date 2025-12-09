---
id: gazebo-fundamentals
title: Gazebo Fundamentals
sidebar_position: 2
description: Core Gazebo architecture, world files, physics engines, and plugin system.
keywords: [gazebo, sdf, world files, physics engine, gazebo plugins, ode, bullet]
---

# Gazebo Fundamentals

## Architecture

Gazebo uses a **client-server model**:
- **gzserver**: Physics simulation (headless, can run without GUI)
- **gzclient**: 3D visualization (connects to server)

This separation allows running simulations on powerful servers while viewing from lightweight clients.

## World Files (SDF Format)

World files use **SDF (Simulation Description Format)** to define environments:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office">

    <!-- Physics engine settings -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom model -->
    <model name="box">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Physics Engines

Gazebo supports three physics engines:

| Engine | Speed | Accuracy | Best For |
|--------|-------|----------|----------|
| **ODE** | Fast | Moderate | General robotics (default) |
| **Bullet** | Very Fast | Lower | Many objects, soft bodies |
| **DART** | Moderate | High | Precise contacts, manipulation |

**Select engine in world file**:
```xml
<physics type="ode">  <!-- or "bullet" or "dart" -->
  <real_time_update_rate>1000</real_time_update_rate>
  <max_step_size>0.001</max_step_size>
</physics>
```

## Plugin System

Plugins add functionality to models. Common types:

### Model Plugin (per robot)
```xml
<plugin name="joint_controller" filename="libgazebo_ros_joint_state_publisher.so">
  <ros><namespace>/robot</namespace></ros>
  <update_rate>50</update_rate>
</plugin>
```

### Sensor Plugin (camera, LiDAR)
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```

### World Plugin (global)
```xml
<world name="default">
  <plugin name="ros_interface" filename="libgazebo_ros_init.so"/>
</world>
```

## Launch Gazebo with ROS 2

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'worlds/office.world'],
            output='screen'
        )
    ])
```

## Common Commands

```bash
# Launch with GUI
gazebo worlds/empty.world

# Headless (server only)
gzserver worlds/empty.world

# Connect client to existing server
gzclient

# List running models
gz model --list

# Get model pose
gz model -m robot_name -p

# Set model pose
gz model -m robot_name -x 1 -y 2 -z 0.5
```

## Performance Tuning

**Slow simulation fixes**:
1. **Reduce update rate**: `<real_time_update_rate>500</real_time_update_rate>` (from 1000)
2. **Increase step size**: `<max_step_size>0.002</max_step_size>` (from 0.001)
3. **Simplify collision geometry**: Use boxes/cylinders instead of meshes
4. **Disable shadows**: Set `<cast_shadows>false</cast_shadows>` in visuals

## Debugging

**Check if Gazebo is running**:
```bash
gz topic -l  # List Gazebo topics
ros2 topic list  # List ROS 2 topics (should see /clock if bridge active)
```

**View joint states**:
```bash
gz joint -m robot_name -l  # List joints
gz joint -m robot_name -j joint_name  # Get joint state
```

**Reset simulation**:
```bash
gz world -r  # Reset world
gz world -p 1  # Pause
gz world -p 0  # Unpause
```

## Best Practices

1. **Start simple**: Test with empty world, then add complexity
2. **Use primitives**: Boxes/cylinders for collision geometry (faster than meshes)
3. **Static models**: Mark non-moving objects as `<static>true</static>`
4. **Separate visual/collision**: High-detail visual, simple collision
5. **Monitor real-time factor**: `gz stats` (target: 1.0 = real-time)

**Next**: [Spawning Robots in Gazebo](/docs/module-02-simulation/spawning-robots)
