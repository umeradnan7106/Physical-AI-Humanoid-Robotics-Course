---
id: building-packages
title: Building ROS 2 Packages
sidebar_position: 4
description: Master ROS 2 package structure, ament_python build system, colcon workspace management, and dependency handling.
keywords: [ros 2 packages, ament python, colcon build, package.xml, setup.py, workspace, dependencies, ros2 pkg]
reading_time: 13
---

# Building ROS 2 Packages

**A ROS 2 package is the fundamental unit of code organization.** Just as Python has modules and JavaScript has npm packages, ROS 2 uses packages to group related nodes, launch files, and configuration. Understanding package structure is essential for building maintainable robot systems.

## What is a Package?

A **package** is a directory containing:
- Python or C++ source code for nodes
- Configuration files (`package.xml`, `setup.py`)
- Launch files, URDF models, and parameters
- Tests and documentation

**One package = one specific functionality.** Examples:
- `camera_driver`: Interfaces with camera hardware
- `object_detection`: Detects objects in images
- `navigation`: Plans paths and controls motors

**Why packages?**
- **Modularity**: Each package has clear purpose and boundaries
- **Reusability**: Share packages across projects via GitHub
- **Dependency management**: Declare what your package needs (e.g., `sensor_msgs`, `opencv`)
- **Build isolation**: One package's build issues don't break others

## Package Types

ROS 2 supports three package types:

| Type | Language | Build System | Use Case |
|------|----------|--------------|----------|
| **ament_python** | Python | setuptools | Most nodes, simple applications |
| **ament_cmake** | C++ | CMake | Performance-critical code, hardware drivers |
| **ament_cmake** (mixed) | C++ + Python | CMake | Combine C++ libraries with Python wrappers |

**This chapter focuses on `ament_python`** (90% of humanoid control code uses Python).

## Anatomy of a Python Package

### Directory Structure

```
my_robot_pkg/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python installation configuration
├── setup.cfg            # Python build settings
├── resource/            # Package marker files
│   └── my_robot_pkg
├── my_robot_pkg/        # Python module (same name as package)
│   ├── __init__.py
│   ├── my_node.py       # Node source files
│   └── utils.py         # Helper modules
├── launch/              # Launch files (optional)
│   └── my_robot.launch.py
├── urdf/                # Robot models (optional)
│   └── my_robot.urdf
├── config/              # Parameters (optional)
│   └── params.yaml
└── test/                # Unit tests (recommended)
    └── test_my_node.py
```

**Key principle**: Package name (`my_robot_pkg`) must match Python module directory name.

### Essential Files Explained

#### 1. `package.xml` - Package Metadata

The `package.xml` file declares dependencies and metadata.

**Example**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>1.0.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- Testing dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Key sections**:
- `<name>`: Package identifier (must be unique in workspace)
- `<version>`: Semantic versioning (MAJOR.MINOR.PATCH)
- `<buildtool_depend>`: Build system (always `ament_python` for Python packages)
- `<depend>`: Runtime dependencies (other ROS 2 packages)
- `<test_depend>`: Testing tools
- `<export><build_type>`: Tells `colcon` how to build this package

**Common dependencies**:
- `rclpy`: ROS 2 Python client library (always needed)
- `std_msgs`: Standard message types (String, Int32, Bool, etc.)
- `sensor_msgs`: Sensor data (Image, LaserScan, Imu, etc.)
- `geometry_msgs`: Poses, transforms (Pose, Twist, Transform, etc.)
- `nav_msgs`: Navigation (Odometry, Path, OccupancyGrid)

#### 2. `setup.py` - Python Installation

The `setup.py` file tells Python how to install your package.

**Example**:
```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_robot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/my_robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_pkg.my_node:main',
            'teleop_node = my_robot_pkg.teleop:main',
        ],
    },
)
```

**Key sections**:
- `packages`: List of Python modules to install (usually just `[package_name]`)
- `data_files`: Non-Python files (launch, URDF, config) - installed to `share/`
- `entry_points`: Executable scripts that users can run with `ros2 run`

**Entry points syntax**:
```python
'executable_name = package.module:function'
```

Example: `'my_node = my_robot_pkg.my_node:main'` creates an executable `my_node` that calls `main()` function in `my_robot_pkg/my_node.py`.

#### 3. `setup.cfg` - Build Settings

**Minimal `setup.cfg`**:
```ini
[develop]
script_dir=$base/lib/my_robot_pkg
[install]
install_scripts=$base/lib/my_robot_pkg
```

This ensures executables install to the correct location. Rarely needs modification.

## Creating a Package

### Method 1: Manual Creation

**Step-by-step**:

```bash
cd ~/ros2_ws/src
mkdir my_robot_pkg
cd my_robot_pkg

# Create package.xml
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>1.0.0</version>
  <description>My robot package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF

# Create setup.py (see example above)
# Create setup.cfg (see example above)
# Create Python module directory
mkdir my_robot_pkg
touch my_robot_pkg/__init__.py
mkdir resource
touch resource/my_robot_pkg
```

### Method 2: Using `ros2 pkg create` (Recommended)

**Automatic boilerplate generation**:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg \
  --dependencies rclpy std_msgs sensor_msgs \
  --maintainer-name "Your Name" \
  --maintainer-email "you@example.com"
```

**What this does**:
- Creates directory structure
- Generates `package.xml` with specified dependencies
- Creates `setup.py` and `setup.cfg` templates
- Creates Python module directory with `__init__.py`

**Always use `ros2 pkg create` for new packages.** It prevents common mistakes.

## Building with Colcon

**Colcon** is the ROS 2 build tool. It discovers packages, resolves dependencies, and builds them in correct order.

### Build Commands

**Build entire workspace**:
```bash
cd ~/ros2_ws
colcon build
```

**Build specific package**:
```bash
colcon build --packages-select my_robot_pkg
```

**Build with debug symbols** (for GDB debugging):
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**Clean build** (forces rebuild):
```bash
rm -rf build/ install/ log/
colcon build
```

### Build Output

After `colcon build`, three directories appear:

```
ros2_ws/
├── build/         # Intermediate build files (can delete)
├── install/       # Installed packages (ready to use)
│   ├── setup.bash # Source this file
│   └── my_robot_pkg/
│       ├── lib/my_robot_pkg/my_node  # Executables
│       └── share/my_robot_pkg/       # Data files
├── log/           # Build logs
└── src/           # Source code
```

**Important**: Always source `install/setup.bash` after building:
```bash
source ~/ros2_ws/install/setup.bash
```

This updates environment variables so `ros2 run` can find your package.

## Workspace Sourcing

**Sourcing** loads ROS 2 environment variables.

**Three levels of sourcing**:

1. **ROS 2 installation**:
```bash
source /opt/ros/humble/setup.bash
```
Loads ROS 2 base packages (`rclpy`, `std_msgs`, etc.)

2. **Workspace overlay**:
```bash
source ~/ros2_ws/install/setup.bash
```
Loads your custom packages *on top of* ROS 2 base

3. **Permanent sourcing** (add to `~/.bashrc`):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Overlay principle**: Later sources override earlier ones. This lets you develop custom versions of ROS 2 packages.

## Dependency Management

### Adding Dependencies

**Runtime dependency** (needed when running):
```xml
<depend>geometry_msgs</depend>
```

**Build-only dependency**:
```xml
<build_depend>rosidl_default_generators</build_depend>
```

**Test-only dependency**:
```xml
<test_depend>python3-pytest</test_depend>
```

### Installing Dependencies with rosdep

**rosdep** automatically installs dependencies declared in `package.xml`.

**First time setup**:
```bash
sudo rosdep init
rosdep update
```

**Install dependencies for workspace**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**What this does**:
- Reads all `package.xml` files in `src/`
- Installs missing dependencies (apt packages for Ubuntu)
- `-r`: Continue despite errors
- `-y`: Auto-confirm installations

**Always run `rosdep install` before building a new workspace.**

## Common Build Issues

### Issue: `Package 'my_robot_pkg' not found`
**Cause**: Forgot to source workspace
**Fix**:
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue: `No module named 'my_robot_pkg'`
**Cause**: Python module directory missing or misnamed
**Fix**: Ensure `my_robot_pkg/my_robot_pkg/__init__.py` exists

### Issue: `Setup script exited with error`
**Cause**: Syntax error in `setup.py`
**Fix**: Check `log/latest_build/my_robot_pkg/stdout_stderr.log` for Python traceback

### Issue: Executables not found after build
**Cause**: Missing or incorrect `entry_points` in `setup.py`
**Fix**: Verify `entry_points` syntax:
```python
entry_points={
    'console_scripts': [
        'node_name = package.module:main',
    ],
},
```

## Best Practices

1. **One package, one purpose**: Don't mix unrelated nodes in same package
2. **Meaningful names**: `humanoid_controller`, not `pkg1`
3. **Version control**: Initialize git in workspace root, not individual packages
4. **Document dependencies**: Keep `package.xml` up-to-date
5. **Test before commit**: Run `colcon build` and `colcon test` before pushing
6. **Use rosdep**: Let `rosdep` manage system dependencies

## Summary

- **Package structure**: `package.xml` (metadata), `setup.py` (Python install), source code in module directory
- **Creating packages**: Use `ros2 pkg create --build-type ament_python` for boilerplate
- **Building**: `colcon build` compiles packages, creates `install/` directory
- **Sourcing**: `source install/setup.bash` loads packages into environment
- **Dependencies**: Declare in `<depend>` tags, install with `rosdep install`
- **Entry points**: Define executables in `setup.py` `console_scripts`

**Next**: Learn to orchestrate multiple nodes with [Launch Files](/docs/module-01-ros2/launch-files).
