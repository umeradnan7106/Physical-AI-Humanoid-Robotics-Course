---
id: sensor-integration
title: Sensor Integration
sidebar_position: 4
description: Add camera, LiDAR, and IMU sensors to Gazebo robots.
keywords: [gazebo camera, gazebo lidar, imu sensor, sensor plugins]
---

# Sensor Integration

## Camera Plugin

Add to URDF link:

```xml
<link name="camera_link">
  <visual>
    <geometry><box size="0.01 0.05 0.02"/></geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes**: `/robot/head_camera/image_raw` (sensor_msgs/Image)

## LiDAR Plugin (2D)

```xml
<link name="lidar_link"/>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes**: `/robot/scan` (sensor_msgs/LaserScan)

## IMU Plugin

```xml
<gazebo reference="base_link">
  <sensor name="imu" type="imu">
    <update_rate>100</update_rate>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
      </ros>
      <frame_name>base_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes**: `/robot/imu` (sensor_msgs/Imu)

## Depth Camera

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>8.0</far>
      </clip>
    </camera>
    <plugin name="depth_controller" filename="libgazebo_ros_camera.so">
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes**:
- `/depth_camera/image_raw` (RGB)
- `/depth_camera/depth/image_raw` (depth map)
- `/depth_camera/points` (point cloud)

## View Sensor Data

**Camera**:
```bash
ros2 run rqt_image_view rqt_image_view /robot/head_camera/image_raw
```

**LiDAR in RViz**:
```bash
rviz2
# Add LaserScan display, topic: /robot/scan
```

**IMU**:
```bash
ros2 topic echo /robot/imu
```

## Sensor Noise

Add realistic noise:

```xml
<sensor name="camera" type="camera">
  <camera>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
</sensor>
```

**Next**: [Unity Integration](/docs/module-02-simulation/unity-robotics)
