---
id: project-isaac-navigation
title: Project - Isaac Navigation
sidebar_position: 6
description: Build autonomous warehouse navigation using cuVSLAM, Nav2, and Isaac ROS perception.
keywords: [isaac sim project, warehouse navigation, cuvslam, nav2, apriltag]
---

# Project: Autonomous Warehouse Navigation

## Objective

Build a humanoid robot that autonomously navigates a warehouse environment using **vision-only** localization and GPU-accelerated perception.

**Goal**: Navigate from loading dock to 3 target shelves, detect AprilTag markers at each location, return to start. Success = 90%+ completion rate over 10 trials.

## Requirements

### 1. Isaac Sim Environment
- **Warehouse scene**: Use `Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd`
- **3 AprilTag targets**: Place tags (ID 0, 1, 2) on shelves at distances 5m, 10m, 15m
- **Dynamic obstacles**: Add 2 moving actors (simulated forklifts)
- **Lighting**: Realistic warehouse lighting (mix of ambient + overhead lights)

### 2. Robot Configuration
- **Stereo camera**: 1280×720, 30 Hz, baseline 0.06m
- **cuVSLAM**: For continuous localization
- **AprilTag detector**: For target identification
- **Nav2 stack**: Path planning and obstacle avoidance

### 3. Navigation Logic
- **Mission planner**: Visit shelves in sequence (0 → 1 → 2 → home)
- **Tag verification**: Detect correct AprilTag ID before marking goal complete
- **Obstacle avoidance**: Dynamic replanning when path blocked
- **Recovery behaviors**: Handle stuck/lost states

### 4. Deliverables
- Launch file starting all components
- Recorded rosbag (2+ minutes)
- Performance report (success rate, completion time)
- Video demonstration

## Implementation

### Step 1: Load Warehouse in Isaac Sim

**Launch Isaac Sim**:
```bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

**Load warehouse** (File → Open):
```
Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd
```

**Add AprilTags to scene**:
```python
# In Isaac Sim Script Editor
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf

# Tag 0 on shelf at (5, 0, 1.5)
add_reference_to_stage(
    usd_path="/Isaac/Props/AprilTag/apriltag_36h11_00000.usd",
    prim_path="/World/Tags/tag_0"
)
from pxr import UsdGeom
xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Tags/tag_0"))
xform.AddTranslateOp().Set(Gf.Vec3d(5.0, 0.0, 1.5))

# Repeat for tags 1 and 2 at different positions
```

### Step 2: Import Humanoid with Stereo Camera

**Add stereo rig to URDF** (modify from Module 1):
```xml
<!-- Left camera -->
<link name="camera_left">
  <visual><geometry><box size="0.01 0.03 0.02"/></geometry></visual>
</link>
<joint name="camera_left_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_left"/>
  <origin xyz="0.05 0.03 0" rpy="0 0 0"/>
</joint>

<!-- Right camera -->
<link name="camera_right">
  <visual><geometry><box size="0.01 0.03 0.02"/></geometry></visual>
</link>
<joint name="camera_right_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_right"/>
  <origin xyz="0.05 -0.03 0" rpy="0 0 0"/>
</joint>
```

**Import in Isaac Sim** (follow isaac-sim-setup.md instructions).

### Step 3: Mission Planner Node

**mission_planner.py**:
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray, '/tag_detections', self.tag_callback, 10
        )

        self.goals = [
            {'x': 5.0, 'y': 0.0, 'tag_id': 0},
            {'x': 10.0, 'y': 2.0, 'tag_id': 1},
            {'x': 15.0, 'y': -1.0, 'tag_id': 2},
            {'x': 0.0, 'y': 0.0, 'tag_id': -1}  # Home
        ]
        self.current_goal_idx = 0
        self.tag_detected = False

        self.send_next_goal()

    def tag_callback(self, msg):
        expected_id = self.goals[self.current_goal_idx]['tag_id']
        for detection in msg.detections:
            if detection.id == expected_id:
                self.tag_detected = True
                self.get_logger().info(f'Tag {expected_id} detected!')

    def send_next_goal(self):
        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info('Mission complete!')
            return

        goal = self.goals[self.current_goal_idx]
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.pose.position.x = goal['x']
        nav_goal.pose.pose.position.y = goal['y']
        nav_goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal {self.current_goal_idx}: {goal}')
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.tag_detected or self.goals[self.current_goal_idx]['tag_id'] == -1:
            self.current_goal_idx += 1
            self.tag_detected = False
            self.send_next_goal()
        else:
            self.get_logger().warn('Goal reached but tag not detected, retrying...')
            self.send_next_goal()
```

### Step 4: Master Launch File

**warehouse_mission.launch.py**:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # cuVSLAM
        IncludeLaunchDescription('vslam.launch.py'),

        # AprilTag detector
        IncludeLaunchDescription('apriltag.launch.py'),

        # Nav2 stack
        IncludeLaunchDescription('nav2_isaac.launch.py'),

        # Mission planner
        Node(
            package='warehouse_nav',
            executable='mission_planner',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'warehouse_nav.rviz']
        )
    ])
```

**Run**:
```bash
# Terminal 1: Isaac Sim (manual launch)
# Terminal 2: Launch stack
ros2 launch warehouse_nav warehouse_mission.launch.py

# Terminal 3: Record data
ros2 bag record -a -o warehouse_mission
```

## Testing & Evaluation

**Success criteria checklist**:
- [ ] Robot spawns at loading dock (0, 0, 0)
- [ ] cuVSLAM initializes (trajectory visible in RViz)
- [ ] Navigates to shelf 1, detects Tag ID 0
- [ ] Navigates to shelf 2, detects Tag ID 1
- [ ] Navigates to shelf 3, detects Tag ID 2
- [ ] Returns to loading dock
- [ ] Avoids dynamic obstacles (forklifts)
- [ ] Completes mission in < 5 minutes
- [ ] 9/10 trials successful (90%+ rate)

**Performance metrics**:
```bash
ros2 topic echo /mission_planner/status
# Logs: start time, goal timestamps, errors
```

**Analyze rosbag**:
```bash
ros2 bag info warehouse_mission
# Check topics recorded: /visual_slam/tracking/odometry, /tag_detections, /cmd_vel

ros2 bag play warehouse_mission --topics /visual_slam/tracking/odometry
# Replay trajectory for debugging
```

## Extensions

1. **Multi-robot coordination**: Add 2nd humanoid, coordinate paths
2. **Object manipulation**: Pick item from shelf after navigation
3. **Semantic mapping**: Label shelves with detected objects
4. **Voice commands**: Integrate VLA for "go to shelf 2"

**Next Module**: [Vision-Language-Action (VLA) Models](/docs/module-04-vla/index) - Connect language to robot actions.

## Troubleshooting

**VSLAM loses tracking**:
- Reduce robot max velocity to 0.3 m/s
- Add more visual features to warehouse (posters, labels)

**Tags not detected**:
- Check camera focal length matches `camera_info`
- Print tags at exact 16cm size
- Ensure proper lighting (avoid glare)

**Nav2 path blocked**:
- Increase costmap update frequency to 10 Hz
- Tune inflation radius in `nav2_params.yaml`
