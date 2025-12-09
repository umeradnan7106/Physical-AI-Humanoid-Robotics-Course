---
id: nav2-navigation
title: Nav2 Navigation
sidebar_position: 4
description: Integrate ROS 2 Nav2 stack with Isaac Sim for autonomous navigation.
keywords: [nav2, navigation stack, costmap, path planning, behavior tree]
---

# Nav2 Navigation Stack

**From pose to goal—autonomously.**

## What is Nav2?

**Nav2** (Navigation 2) is ROS 2's official navigation framework:
- **Path planning**: A* or Theta* algorithms to compute collision-free paths
- **Costmaps**: 2D grids marking obstacles (from LiDAR/camera/VSLAM)
- **Controller**: Follows planned paths with velocity commands
- **Behavior trees**: Handle recovery actions (stuck, lost, etc.)

**Inputs**: Odometry (from cuVSLAM), sensor data (LiDAR/depth camera)
**Outputs**: `/cmd_vel` (geometry_msgs/Twist) for robot movement

## Install Nav2

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Configure Nav2 Parameters

**nav2_params.yaml**:
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /visual_slam/tracking/odometry
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: depth_camera
        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

## Launch Nav2 with Isaac Sim

**nav2_isaac.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('my_nav'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([
        # Nav2 lifecycle nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }],
            output='screen'
        )
    ])
```

**Run**:
```bash
# Terminal 1: Isaac Sim with robot + cuVSLAM
ros2 launch my_vslam isaac_vslam.launch.py

# Terminal 2: Nav2 stack
ros2 launch my_nav nav2_isaac.launch.py

# Terminal 3: RViz
rviz2 -d $(ros2 pkg prefix my_nav)/share/my_nav/rviz/nav2.rviz
```

## Set Navigation Goal

**Via RViz**:
1. Click **"2D Goal Pose"** button
2. Click destination on map
3. Robot plans path and moves autonomously

**Via command line**:
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'},
    pose: {position: {x: 5.0, y: 2.0, z: 0.0},
           orientation: {w: 1.0}}}" --once
```

**Via Python action client**:
```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('nav_goal_sender')
action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.pose.position.x = 5.0
goal_msg.pose.pose.position.y = 2.0
goal_msg.pose.pose.orientation.w = 1.0

action_client.wait_for_server()
future = action_client.send_goal_async(goal_msg)
rclpy.spin_until_future_complete(node, future)
```

## Visualize Costmaps in RViz

**Add displays**:
- **Map**: Topic `/map` (if using SLAM Toolbox)
- **Local Costmap**: Topic `/local_costmap/costmap`
- **Global Costmap**: Topic `/global_costmap/costmap`
- **Path**: Topic `/plan` (planned path in green)
- **Robot Footprint**: Topic `/local_costmap/published_footprint`

**Color meanings**:
- **Red**: Obstacles (cost = 254)
- **Blue**: Free space (cost = 0)
- **Purple**: Inflation layer (cost = 1-253)
- **Black**: Unknown space (cost = 255)

## Tune Navigation Performance

**Robot moves too slowly**:
- Increase `max_vel_x` in `nav2_params.yaml`
- Reduce `robot_radius` if overly conservative

**Robot gets stuck**:
- Increase `tolerance` in planner config
- Enable recovery behaviors: `use_rotate_recovery: True`

**Jittery movement**:
- Reduce `controller_frequency` to 10 Hz
- Increase `min_vel_x` threshold

**Collisions**:
- Increase `robot_radius` or inflation layer radius
- Check sensor data: `ros2 topic echo /camera/depth/points`

## Nav2 Behavior Trees

**Default tree** (`navigate_to_pose_w_replanning_and_recovery.xml`):
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <SequenceStar name="RecoveryActions">
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <Spin spin_dist="1.57"/>
        <Wait wait_duration="5"/>
      </SequenceStar>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

**Recovery sequence**: If navigation fails → Clear costmap → Spin 90° → Wait 5s → Retry

**Next**: [Isaac ROS Perception](/docs/module-03-isaac/isaac-ros-perception)
