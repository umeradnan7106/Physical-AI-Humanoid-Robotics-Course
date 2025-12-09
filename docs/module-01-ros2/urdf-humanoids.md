---
id: urdf-humanoids
title: URDF for Humanoid Robots
sidebar_position: 6
description: Build complete humanoid robot models using URDF and xacro - links, joints, kinematics, and RViz visualization.
keywords: [urdf, xacro, robot modeling, humanoid urdf, links, joints, rviz, robot state publisher, collision geometry, visual geometry]
reading_time: 16
---

# URDF for Humanoid Robots

**You can't control what you can't describe.** A humanoid robot has 20-40 joints (shoulders, elbows, hips, knees, ankles). Before writing control code, you must define the robot's structure: which parts connect to which, how they move, and where they are in space. URDF (Unified Robot Description Format) is ROS 2's language for robot geometry and kinematics.

## What is URDF?

**URDF** is an XML-based format that describes:
- **Links**: Rigid bodies (torso, upper arm, forearm, thigh, shin)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Kinematics**: How joints move and their limits
- **Visual geometry**: 3D meshes or primitive shapes for display
- **Collision geometry**: Simplified shapes for physics simulation
- **Inertia**: Mass distribution for dynamics

**Why URDF?**
- **Visualization**: See your robot in RViz before building hardware
- **Simulation**: Run in Gazebo or Isaac Sim with accurate physics
- **Control**: Motion planners need joint limits and link transforms
- **Standardization**: Same format for all ROS 2 robots

## URDF Basics

### Minimal Robot: Two Links and One Joint

**Example**: Simple pendulum (base + arm connected by revolute joint)

```xml
<?xml version="1.0"?>
<robot name="simple_pendulum">

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.02"/>
      </geometry>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

**Key concepts**:
- `<link>`: Defines a rigid body with visual geometry
- `<joint>`: Connects two links with motion constraints
- `<origin>`: Position and orientation (xyz in meters, rpy in radians)
- `<axis>`: Rotation axis for revolute joints (y-axis = [0 1 0])
- `<limit>`: Joint range (±90° here), max torque, max velocity

### Link Structure

Every link can have four components:

```xml
<link name="forearm">

  <!-- Visual: What you see in RViz -->
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.03"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>

  <!-- Collision: Simplified geometry for physics -->
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.04"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </collision>

  <!-- Inertial: Mass and moment of inertia -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <inertia ixx="0.0083" ixy="0" ixz="0"
             iyy="0.0083" iyz="0" izz="0.0005"/>
  </inertial>

</link>
```

**Component purposes**:
- **Visual**: High-detail meshes for realistic rendering (can use `.stl`, `.dae`, `.obj` files)
- **Collision**: Low-polygon shapes for fast physics (spheres, boxes, cylinders)
- **Inertial**: Required for Gazebo simulation (use MeshLab or SolidWorks to compute)
- **Material**: RGBA colors for visualization (optional)

**Geometry types**:
- `<box size="x y z"/>` - Rectangular prism
- `<cylinder length="L" radius="r"/>` - Circular cylinder
- `<sphere radius="r"/>` - Perfect sphere
- `<mesh filename="package://my_robot/meshes/arm.stl"/>` - Custom 3D model

### Joint Types

| Type | Description | Degrees of Freedom | Use Case |
|------|-------------|-------------------|----------|
| **revolute** | Rotates around axis with limits | 1 | Elbows, knees, shoulders |
| **continuous** | Rotates around axis infinitely | 1 | Wheels, turrets |
| **prismatic** | Slides along axis with limits | 1 | Linear actuators, grippers |
| **fixed** | No movement | 0 | Sensors, cameras attached to links |
| **floating** | Free movement in 3D space | 6 | Base of mobile robots |
| **planar** | Moves in XY plane | 2 | Rarely used |

**Example: Knee joint (revolute with limits)**

```xml
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.356" effort="100" velocity="2.0"/>
  <!-- Limit: 0° to 135° (knee bends forward only) -->
</joint>
```

**Example: Camera mount (fixed joint)**

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.1" rpy="0 0.2 0"/>
  <!-- Camera tilted 11.5° downward -->
</joint>
```

## Xacro: Reusable URDF Macros

**Problem**: URDF is verbose. A humanoid with two arms requires duplicating 14 links and 12 joints (left arm + right arm).

**Solution**: **xacro** (XML Macros) adds:
- Variables and constants
- Math operations
- Macros (functions that generate URDF snippets)
- File includes

### Xacro Syntax

**Variables**:
```xml
<xacro:property name="arm_length" value="0.3"/>
<xacro:property name="arm_radius" value="0.03"/>
```

**Math**:
```xml
<origin xyz="0 0 ${arm_length / 2}" rpy="0 0 0"/>
<!-- Evaluates to xyz="0 0 0.15" -->
```

**Macros**:
```xml
<xacro:macro name="arm_link" params="prefix length radius">
  <link name="${prefix}_arm">
    <visual>
      <geometry>
        <cylinder length="${length}" radius="${radius}"/>
      </geometry>
      <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
    </visual>
  </link>
</xacro:macro>

<!-- Usage -->
<xacro:arm_link prefix="left" length="0.3" radius="0.03"/>
<xacro:arm_link prefix="right" length="0.3" radius="0.03"/>
<!-- Generates left_arm and right_arm links -->
```

### Complete Humanoid Example (Xacro)

**File**: `my_robot_pkg/urdf/humanoid.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Constants -->
  <xacro:property name="torso_height" value="0.5"/>
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="leg_length" value="0.4"/>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${torso_width} 0.2 ${torso_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Macro for arm (upper + lower + hand) -->
  <xacro:macro name="arm" params="prefix reflect">

    <!-- Shoulder joint -->
    <joint name="${prefix}_shoulder" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0 ${reflect * torso_width/2} ${torso_height/2 - 0.05}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
    </joint>

    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.03"/>
        </geometry>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
      </visual>
    </link>

    <!-- Elbow joint -->
    <joint name="${prefix}_elbow" type="revolute">
      <parent link="${prefix}_upper_arm"/>
      <child link="${prefix}_lower_arm"/>
      <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.356" effort="30" velocity="1.5"/>
    </joint>

    <link name="${prefix}_lower_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.025"/>
        </geometry>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
      </visual>
    </link>

  </xacro:macro>

  <!-- Instantiate left and right arms -->
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>

  <!-- Macro for leg (thigh + shin + foot) -->
  <xacro:macro name="leg" params="prefix reflect">

    <!-- Hip joint -->
    <joint name="${prefix}_hip" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${reflect * 0.1} ${-torso_height/2}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>

    <link name="${prefix}_thigh">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="0.04"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length/2}" rpy="0 0 0"/>
        <material name="black"><color rgba="0 0 0 1"/></material>
      </visual>
    </link>

    <!-- Knee joint -->
    <joint name="${prefix}_knee" type="revolute">
      <parent link="${prefix}_thigh"/>
      <child link="${prefix}_shin"/>
      <origin xyz="0 0 ${-leg_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.356" effort="100" velocity="1.5"/>
    </joint>

    <link name="${prefix}_shin">
      <visual>
        <geometry>
          <cylinder length="${leg_length}" radius="0.035"/>
        </geometry>
        <origin xyz="0 0 ${-leg_length/2}" rpy="0 0 0"/>
        <material name="black"><color rgba="0 0 0 1"/></material>
      </visual>
    </link>

  </xacro:macro>

  <!-- Instantiate left and right legs -->
  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>

  <!-- Head -->
  <joint name="neck" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height/2 + 0.05}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin"><color rgba="0.95 0.8 0.7 1"/></material>
    </visual>
  </link>

</robot>
```

**This humanoid has**:
- **10 DOF**: 2 shoulders + 2 elbows + 2 hips + 2 knees + 1 neck + 1 head = 10 joints
- **11 links**: torso, 2 upper arms, 2 lower arms, 2 thighs, 2 shins, head
- **Symmetric design**: Macros generate identical left/right limbs

## Visualizing in RViz

### Step 1: Convert Xacro to URDF

Xacro files must be converted to plain URDF before use:

```bash
cd ~/ros2_ws/src/my_robot_pkg/urdf
ros2 run xacro xacro humanoid.urdf.xacro > humanoid.urdf
```

**Or use xacro in launch file** (automatic conversion):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf.xacro')

    # Process xacro to URDF
    robot_description = xacro.process_file(xacro_file).toxml()

    # Robot state publisher (publishes TF transforms)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher GUI (control joint angles)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'humanoid.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
```

### Step 2: Run Launch File

```bash
ros2 launch my_robot_pkg view_humanoid.launch.py
```

**What happens**:
1. **robot_state_publisher**: Reads URDF, publishes `/tf` transforms (link positions in 3D space)
2. **joint_state_publisher_gui**: Opens sliders to control each joint angle
3. **RViz**: Visualizes robot with live updates as you move sliders

**In RViz**:
- Add **RobotModel** display (shows URDF)
- Set **Fixed Frame** to `base_link`
- Move sliders in joint_state_publisher_gui → robot moves in RViz

## Best Practices

### 1. Name Consistency
Use standard naming conventions:
- **Links**: `base_link`, `left_upper_arm`, `right_shin`, `head`
- **Joints**: `left_shoulder`, `right_knee`, `neck`
- **Frames**: Match link names for TF tree clarity

### 2. Origin Placement
Place joint origins at the **physical rotation point**, not link centers.

**Bad** (joint at link center):
```xml
<joint name="elbow">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- Elbow rotates at wrong point -->
</joint>
```

**Good** (joint at anatomical elbow):
```xml
<joint name="elbow">
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <!-- Elbow at end of upper arm -->
</joint>
```

### 3. Collision vs Visual Geometry
**Visual**: Can be complex meshes (10k+ polygons)
**Collision**: Must be simple (spheres, boxes, convex hulls with `<100` polygons)

**Why**: Physics engines (Gazebo, Isaac Sim) check collisions millions of times per second. Complex collision geometry = simulation slowdown.

**Example**:
```xml
<visual>
  <geometry>
    <mesh filename="package://my_robot/meshes/hand_detailed.stl"/>
  </geometry>
</visual>
<collision>
  <geometry>
    <box size="0.1 0.08 0.05"/>  <!-- Simple box approximation -->
  </geometry>
</collision>
```

### 4. Inertia Computation
Never guess inertia tensors. Use tools:
- **MeshLab**: Open source mesh analysis tool
- **SolidWorks/Fusion 360**: CAD software exports URDF with accurate inertia
- **PyBullet**: Compute inertia from mesh automatically

**Placeholder for testing** (use only in early prototypes):
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

### 5. Joint Limits
Set realistic limits based on hardware specs or human anatomy:

**Human joint ranges** (approximate):
- Shoulder pitch: -60° to 180°
- Elbow: 0° to 135°
- Hip pitch: -90° to 90°
- Knee: 0° to 135°
- Ankle: -45° to 45°

## Troubleshooting

### Issue: RViz shows "No transform from [link] to [base_link]"
**Cause**: `robot_state_publisher` not running or URDF has disconnected links
**Fix**:
```bash
ros2 node list  # Check if robot_state_publisher is running
ros2 run tf2_tools view_frames  # Generate TF tree PDF
evince frames.pdf  # View to find missing connections
```

### Issue: Joint sliders don't move robot in RViz
**Cause**: `joint_state_publisher_gui` not publishing to correct topic
**Fix**:
```bash
ros2 topic list  # Verify /joint_states exists
ros2 topic echo /joint_states  # Check messages are publishing
```

### Issue: Xacro conversion fails with "undefined property"
**Cause**: Math expression uses undefined variable
**Fix**: Ensure all `<xacro:property>` declarations come **before** usage

### Issue: Links overlap or have wrong positions
**Cause**: Incorrect `<origin>` values in joints
**Fix**: Double-check xyz coordinates. Use RViz measurement tools to verify link lengths.

## Summary

- **URDF**: XML format defining robot structure (links, joints, geometry, inertia)
- **Links**: Rigid bodies with visual, collision, and inertial properties
- **Joints**: Connections between links (revolute, prismatic, fixed, continuous)
- **Xacro**: Macro language for reusable URDF (variables, math, macros)
- **robot_state_publisher**: Publishes TF transforms from URDF
- **joint_state_publisher_gui**: Control joint angles with sliders
- **RViz**: Visualize robot model in 3D
- **Best practices**: Accurate origins, simple collision geometry, realistic joint limits

**Next**: Apply everything you've learned in the [Module Project: ROS 2 Package](/docs/module-01-ros2/project-ros2-package).
