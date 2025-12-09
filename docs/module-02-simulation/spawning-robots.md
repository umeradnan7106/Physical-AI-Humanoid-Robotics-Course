---
id: spawning-robots
title: Spawning Robots in Simulation
sidebar_position: 3
description: Load URDF robots into Gazebo with plugins and ros2_control.
keywords: [gazebo spawn, urdf gazebo, gazebo_ros_control, ros2_control]
---

# Spawning Robots in Gazebo

## Add Gazebo Tags to URDF

Extend your URDF with Gazebo-specific elements:

```xml
<robot name="humanoid">

  <!-- Existing URDF links and joints -->

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient -->
    <mu2>0.8</mu2>
  </gazebo>

  <!-- Joint transmission for ros2_control -->
  <transmission name="left_shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_shoulder">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_shoulder_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```

## ros2_control Plugin

Add controller interface to URDF:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**controllers.yaml**:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - left_shoulder
      - left_elbow
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Spawn Launch File

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot')

    # Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_robot, 'worlds', 'office.world')}.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(pkg_robot, 'urdf', 'humanoid.urdf')).read()}]
    )

    # Controller spawner
    load_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_controllers
    ])
```

## Control Robot

**Send joint commands**:
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['left_shoulder'], points: [{positions: [0.5]}]}" --once
```

**Monitor joint states**:
```bash
ros2 topic echo /joint_states
```

## Common Issues

**Robot falls through ground**:
- Add collision geometry to all links
- Verify ground plane exists in world file

**Joints don't move**:
- Check controller is loaded: `ros2 control list_controllers`
- Verify transmissions in URDF

**Model not visible**:
- Check `/robot_description` topic: `ros2 topic echo /robot_description`
- Verify spawn_entity completed: look for "Successfully spawned entity" in logs

**Next**: [Sensor Integration](/docs/module-02-simulation/sensor-integration)
