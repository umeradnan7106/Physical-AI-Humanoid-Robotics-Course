---
id: project-simulation
title: Project - Autonomous Navigation
sidebar_position: 6
description: Build a complete simulation with autonomous navigation using sensors.
keywords: [gazebo project, autonomous navigation, sensor fusion, obstacle avoidance]
---

# Project: Autonomous Navigation in Gazebo

## Objective

Create a humanoid robot that navigates autonomously in an indoor environment using camera and LiDAR sensors.

**Goal**: Robot moves from start to goal while avoiding obstacles, 90%+ success rate over 20 trials.

## Requirements

### 1. Gazebo World
- **5+ rooms** connected by doorways
- **10+ obstacles** (furniture, walls, moving actors)
- **Realistic lighting** (ambient + directional)
- **Ground plane** with texture

### 2. Sensor-Equipped Robot
- **Camera**: 640x480, 30 Hz, 60° FOV
- **2D LiDAR**: 360 rays, 10m range, 40 Hz
- **IMU**: 100 Hz update rate
- All sensors publish to ROS 2 topics

### 3. Navigation Controller
- **Obstacle detection**: Use LiDAR scan data
- **Path planning**: Simple reactive navigation (no SLAM required)
- **Goal reaching**: Stop within 0.5m of target

### 4. Deliverables
- Launch file starting everything
- Recorded rosbag (30+ seconds)
- Video showing successful navigation

## Implementation Steps

### Step 1: Create World File

```xml
<world name="office_nav">
  <physics type="ode">
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>

  <include><uri>model://sun</uri></include>
  <include><uri>model://ground_plane</uri></include>

  <!-- Rooms -->
  <model name="room1">
    <static>true</static>
    <link name="wall">
      <pose>2 0 1 0 0 0</pose>
      <collision name="collision">
        <geometry><box><size>0.2 10 2</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 10 2</size></box></geometry>
      </visual>
    </link>
  </model>

  <!-- Add more walls, furniture -->
</world>
```

### Step 2: Add Sensors to Humanoid URDF

Follow [Sensor Integration](/docs/module-02-simulation/sensor-integration) to add:
- Camera on head
- LiDAR on torso
- IMU on base_link

### Step 3: Navigation Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.sub = self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_x, self.goal_y = 10.0, 10.0  # Target position

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        min_dist = np.min(ranges[np.isfinite(ranges)])

        cmd = Twist()
        if min_dist < 1.0:
            # Obstacle nearby: turn away
            left_side = np.min(ranges[:len(ranges)//2])
            right_side = np.min(ranges[len(ranges)//2:])
            cmd.angular.z = 0.5 if left_side > right_side else -0.5
        else:
            # Clear path: move forward
            cmd.linear.x = 0.5

        self.pub.publish(cmd)
```

### Step 4: Launch Everything

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with world
        IncludeLaunchDescription('...gazebo.launch.py',
            launch_arguments={'world': 'office_nav.world'}.items()),

        # Spawn robot
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'humanoid', '-topic', '/robot_description']),

        # Navigation controller
        Node(package='sim_project', executable='obstacle_avoidance'),

        # RViz
        Node(package='rviz2', executable='rviz2')
    ])
```

## Testing

```bash
ros2 launch sim_project navigation.launch.py
```

**Check sensors**:
```bash
ros2 topic hz /robot/scan  # Should show ~40 Hz
ros2 topic hz /robot/head_camera/image_raw  # ~30 Hz
```

**Record trial**:
```bash
ros2 bag record -a -o trial_01
```

## Acceptance Criteria

- [ ] World loads without errors
- [ ] Robot spawns and stands upright
- [ ] All sensor topics publishing at correct rates
- [ ] Robot avoids obstacles (no collisions)
- [ ] Reaches goal within 60 seconds
- [ ] 18/20 successful trials (90%+)

## Extensions

1. **SLAM**: Add gmapping or cartographer for mapping
2. **Nav2**: Use full navigation stack with path planning
3. **Dynamic obstacles**: Add moving actors
4. **Multiple robots**: Test with 3+ humanoids

**Next Module**: [Isaac Sim for Perception](/docs/module-03-isaac/index)
