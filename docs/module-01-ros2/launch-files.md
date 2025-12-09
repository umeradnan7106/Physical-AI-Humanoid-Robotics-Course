---
id: launch-files
title: Launch Files
sidebar_position: 5
description: Master ROS 2 Python launch files to orchestrate multiple nodes with parameters, namespaces, and composable node configurations.
keywords: [ros 2 launch files, python launch api, parameters, namespaces, composable nodes, launch configuration, ros2 launch]
reading_time: 14
---

# Launch Files

**Launching robots with `ros2 run` one node at a time is tedious.** A typical humanoid robot has 20+ nodes (cameras, controllers, planners, safety monitors). Launch files let you start entire systems with a single command.

## What is a Launch File?

A **launch file** is a Python script that:
- Starts multiple nodes simultaneously
- Passes parameters to nodes (configuration values)
- Sets namespaces (run multiple robots without conflicts)
- Includes other launch files (modular system bringup)
- Handles conditional logic (start different nodes based on arguments)

**Example use case**: Starting a humanoid robot
```bash
ros2 launch my_robot_pkg humanoid.launch.py
```

This one command might start:
- Camera drivers (3 nodes)
- LiDAR driver (1 node)
- Object detector (1 node)
- Motion planner (1 node)
- Joint controller (1 node)
- Safety monitor (1 node)
- RViz visualization (1 node)

**Total**: 9 nodes from one command.

## Launch File Basics

### Minimal Launch File

**File**: `my_robot_pkg/launch/simple.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='my_robot_pkg',
            executable='listener',
            name='listener_node'
        ),
    ])
```

**Run it**:
```bash
ros2 launch my_robot_pkg simple.launch.py
```

**What happens**: Both `talker` and `listener` nodes start simultaneously.

### Launch File Structure

Every launch file must:
1. Import `LaunchDescription` and action classes
2. Define `generate_launch_description()` function
3. Return a `LaunchDescription` containing actions (Node, IncludeLaunchDescription, etc.)

**Actions** are operations to perform (start node, set parameter, run command).

## Node Configuration

### Basic Node Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='controller',
            name='arm_controller',
            output='screen',  # Print logs to terminal
            parameters=[{
                'max_velocity': 1.0,
                'max_acceleration': 0.5,
                'joint_names': ['shoulder', 'elbow', 'wrist']
            }]
        ),
    ])
```

**Parameters explained**:
- `package`: ROS 2 package name
- `executable`: Entry point name (from `setup.py`)
- `name`: Node name (overrides default, must be unique)
- `output`: `'screen'` prints to terminal, `'log'` saves to file
- `parameters`: List of dictionaries with parameter key-value pairs

**Accessing parameters in node**:
```python
class ControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.declare_parameter('max_velocity', 1.0)
        max_vel = self.get_parameter('max_velocity').value
        self.get_logger().info(f'Max velocity: {max_vel}')
```

### Parameters from YAML File

**Parameters file**: `my_robot_pkg/config/robot_params.yaml`

```yaml
arm_controller:
  ros__parameters:
    max_velocity: 1.5
    max_acceleration: 0.8
    joint_names: ['shoulder', 'elbow', 'wrist']
    pid_gains:
      p: 10.0
      i: 0.1
      d: 1.0
```

**Launch file**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='controller',
            name='arm_controller',
            parameters=[config_file]
        ),
    ])
```

**Why YAML files?**
- Easier to edit than Python dictionaries
- Version controlled separately from code
- Reusable across multiple launch files

## Namespaces

**Namespaces** prevent topic name collisions when running multiple instances of the same node.

**Problem**: Two camera nodes both publish to `/camera/image` → conflict

**Solution**: Use namespaces:

```python
Node(
    package='camera_driver',
    executable='camera_node',
    name='left_camera',
    namespace='left',
    remappings=[('/camera/image', '/image')]
),
Node(
    package='camera_driver',
    executable='camera_node',
    name='right_camera',
    namespace='right',
    remappings=[('/camera/image', '/image')]
),
```

**Result**:
- Left camera publishes to `/left/image`
- Right camera publishes to `/right/image`

**Use case**: Multi-robot systems (two humanoids: `/robot1/joint_states`, `/robot2/joint_states`)

## Launch Arguments

**Launch arguments** make launch files configurable from command line.

**Example**: Choose simulation vs real robot

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation or real robot'
    )

    return LaunchDescription([
        use_sim_arg,
        Node(
            package='my_robot_pkg',
            executable='controller',
            parameters=[{
                'use_simulation': LaunchConfiguration('use_sim')
            }]
        ),
    ])
```

**Run with argument**:
```bash
ros2 launch my_robot_pkg robot.launch.py use_sim:=false
```

**In node**:
```python
self.declare_parameter('use_simulation', True)
use_sim = self.get_parameter('use_simulation').value
if use_sim:
    self.get_logger().info('Running in simulation')
else:
    self.get_logger().info('Running on real robot')
```

## Conditional Logic

**Conditional actions** run based on launch arguments.

**Example**: Start RViz only if `rviz:=true`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    return LaunchDescription([
        rviz_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
```

**Run without RViz**:
```bash
ros2 launch my_robot_pkg robot.launch.py rviz:=false
```

## Including Other Launch Files

**Modular launch files** break complex systems into reusable pieces.

**Example structure**:
```
my_robot_pkg/launch/
├── sensors.launch.py       # Start all sensors
├── controllers.launch.py   # Start all controllers
└── full_robot.launch.py    # Include both
```

**`full_robot.launch.py`**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_pkg')

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
        )
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'controllers.launch.py')
        )
    )

    return LaunchDescription([
        sensors_launch,
        controllers_launch,
    ])
```

**Why modular launch files?**
- Test sensors independently: `ros2 launch my_robot_pkg sensors.launch.py`
- Reuse across projects
- Easier debugging (isolate subsystems)

## Composable Nodes

**Composable nodes** run in the same process for faster communication (no network overhead).

**Normal nodes**: Each node = separate process (inter-process communication via DDS)

**Composable nodes**: Multiple nodes in one process (in-process communication via shared memory)

**When to use**: High-frequency communication between nodes (e.g., camera → image processor at 30 Hz)

**Example**:
```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_tools',
                plugin='image_tools::Cam2Image',
                name='cam2image',
            ),
            ComposableNode(
                package='image_tools',
                plugin='image_tools::ImageToCV',
                name='image_to_cv',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

**Performance gain**: 10-100x faster than DDS for local communication.

## Real-World Launch File Example

**Humanoid robot with camera, controller, and RViz**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('my_robot_pkg')
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')

    # Launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    camera_node = Node(
        package='my_robot_pkg',
        executable='camera_driver',
        parameters=[config_file]
    )

    controller_node = Node(
        package='my_robot_pkg',
        executable='joint_controller',
        parameters=[config_file, {'use_simulation': LaunchConfiguration('use_sim')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        use_sim_arg,
        robot_state_publisher,
        camera_node,
        controller_node,
        rviz_node,
    ])
```

**Run**:
```bash
# With RViz and simulation
ros2 launch my_robot_pkg humanoid.launch.py

# Without RViz, real robot
ros2 launch my_robot_pkg humanoid.launch.py rviz:=false use_sim:=false
```

## Debugging Launch Files

### View active nodes:
```bash
ros2 node list
```

### View node parameters:
```bash
ros2 param list /arm_controller
ros2 param get /arm_controller max_velocity
```

### Check if launch file syntax is valid:
```bash
ros2 launch my_robot_pkg robot.launch.py --show-args
```

**Output**: Lists all launch arguments and default values

## Best Practices

1. **Modular design**: Break into sensor, controller, and visualization launch files
2. **Use YAML for parameters**: Easier to modify than hardcoded values
3. **Meaningful argument names**: `use_sim` not `flag1`
4. **Default values**: Provide sensible defaults for all arguments
5. **Document arguments**: Use `description` field in `DeclareLaunchArgument`
6. **Absolute paths**: Use `get_package_share_directory()` for file paths
7. **Test incrementally**: Start with minimal launch file, add complexity gradually

## Summary

- **Launch files**: Python scripts that start multiple nodes with configuration
- **Basic structure**: `generate_launch_description()` returns `LaunchDescription` with actions
- **Node configuration**: Pass parameters via dictionaries or YAML files
- **Namespaces**: Prevent topic collisions (`/robot1/image` vs `/robot2/image`)
- **Arguments**: Make launch files configurable (`use_sim:=true`)
- **Conditional logic**: Start nodes based on arguments (`IfCondition`)
- **Modular design**: Include other launch files with `IncludeLaunchDescription`
- **Composable nodes**: Run multiple nodes in one process for speed

**Next**: Learn robot modeling with [URDF for Humanoids](/docs/module-01-ros2/urdf-humanoids).
