---
id: environment-setup
title: Environment Setup
sidebar_position: 3
description: Step-by-step installation guide for Ubuntu 22.04, ROS 2 Humble, NVIDIA drivers, Gazebo, and development tools.
keywords: [ubuntu installation, ros 2 humble, nvidia drivers, gazebo installation, colcon, python setup, robotics environment]
reading_time: 10
---

# Environment Setup

**This chapter walks you through installing every tool you'll need.** Follow each step in order, verify as you go, and you'll have a complete Physical AI development environment in under 2 hours.

## Prerequisites

Before starting:
- [ ] Ubuntu 22.04 LTS installed (dual-boot or full install)
- [ ] Internet connection (will download ~5-10 GB)
- [ ] Terminal access (Ctrl+Alt+T)

**If you need to install Ubuntu 22.04**: Download ISO from [ubuntu.com](https://ubuntu.com/download/desktop), create bootable USB with Rufus (Windows) or Etcher (macOS), boot from USB, and follow the installer.

## Step 1: Update System Packages

Open a terminal (Ctrl+Alt+T) and run:

```bash
sudo apt update && sudo apt upgrade -y
```

**What this does**: Updates package lists and upgrades installed software to latest versions.

**Time**: 5-10 minutes (depends on internet speed)

## Step 2: Install ROS 2 Humble

ROS 2 Humble is the robotics middleware we'll use throughout this book.

### 2.1 Add ROS 2 Repository

```bash
# Set locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.2 Install ROS 2 Humble Desktop

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

**What this installs**:
- ROS 2 core libraries
- RViz (3D visualization tool)
- rqt (debugging GUI tools)
- Demos and tutorials

**Time**: 10-15 minutes

### 2.3 Install Development Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete -y
```

**What this installs**:
- `colcon`: Build tool for ROS 2 workspaces
- `rosdep`: Dependency management
- `argcomplete`: Tab-completion for ROS 2 commands

### 2.4 Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### 2.5 Source ROS 2 (Add to .bashrc)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**What this does**: Automatically loads ROS 2 environment variables every time you open a terminal.

### 2.6 Verify ROS 2 Installation

```bash
ros2 --version
```

**Expected output**: `ros2 cli version 0.18.x` or similar

```bash
ros2 topic list
```

**Expected output**: Empty list (no errors means ROS 2 is working)

## Step 3: Install Gazebo Classic

Gazebo is the physics simulator for Modules 1-2.

```bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs -y
```

**What this installs**:
- Gazebo Classic 11 (physics engine)
- ROS 2-Gazebo bridge (connects ROS 2 to Gazebo)

**Time**: 5 minutes

### 3.1 Verify Gazebo

```bash
gazebo --version
```

**Expected output**: `Gazebo multi-robot simulator, version 11.x.x`

Launch Gazebo (GUI should open):

```bash
gazebo
```

**Expected**: Gazebo window opens showing empty world. Close it with Ctrl+C in terminal.

## Step 4: Install NVIDIA GPU Drivers (Optional)

**Skip this step if**: You don't have NVIDIA GPU or plan to use cloud for Module 3.

### 4.1 Check GPU Detection

```bash
lspci | grep -i nvidia
```

**Expected output**: Your NVIDIA GPU model (e.g., `NVIDIA Corporation GA106 [GeForce RTX 3060]`)

**If no output**: Either no NVIDIA GPU present or drivers needed.

### 4.2 Install Recommended Driver

```bash
sudo ubuntu-drivers autoinstall
```

**What this does**: Automatically detects and installs the best NVIDIA driver for your GPU.

**Time**: 5-10 minutes

### 4.3 Reboot

```bash
sudo reboot
```

**Why**: NVIDIA drivers require reboot to load kernel modules.

### 4.4 Verify GPU

After reboot, run:

```bash
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.xx.xx    Driver Version: 525.xx.xx    CUDA Version: 12.x  |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|   0  NVIDIA GeForce ... Off  | 00000000:01:00.0  On |                  N/A |
...
```

**If errors**: See [Troubleshooting Guide](/docs/appendices/troubleshooting#nvidia-driver-issues).

## Step 5: Install Python Dependencies

### 5.1 Install pip and virtualenv

```bash
sudo apt install python3-pip python3-venv -y
```

### 5.2 Install Robotics Python Libraries

```bash
pip3 install numpy opencv-python matplotlib scipy transforms3d
```

**What these are**:
- `numpy`: Numerical computing (matrix operations, sensor data processing)
- `opencv-python`: Computer vision (image processing, object detection)
- `matplotlib`: Plotting (visualize sensor data, trajectories)
- `scipy`: Scientific computing (filters, optimization)
- `transforms3d`: 3D rotations and translations

**Time**: 2-3 minutes

### 5.3 Verify Python

```bash
python3 --version
```

**Expected output**: `Python 3.10.x` or `3.11.x`

Test imports:

```bash
python3 -c "import numpy, cv2, matplotlib; print('All imports successful')"
```

**Expected output**: `All imports successful`

## Step 6: Install Development Tools

### 6.1 Install VS Code (Recommended IDE)

```bash
# Add Microsoft GPG key and repository
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg

# Install VS Code
sudo apt update
sudo apt install code -y
```

### 6.2 Install Git

```bash
sudo apt install git -y
```

Configure Git (replace with your info):

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 6.3 Install Useful VS Code Extensions

Launch VS Code:

```bash
code .
```

Install these extensions (Ctrl+Shift+X):
1. **Python** (Microsoft) - Python language support
2. **ROS** (Microsoft) - ROS/ROS 2 syntax highlighting
3. **C/C++** (Microsoft) - For URDF and low-level code
4. **GitLens** - Enhanced Git integration

## Step 7: Clone Code Repository

```bash
# Navigate to home directory
cd ~

# Clone the book's code examples
git clone https://github.com/umeradnan7106/physical-ai-code-examples.git
cd physical-ai-code-examples
```

### 7.1 Install Project Dependencies

```bash
pip3 install -r requirements.txt
```

**What this installs**: PyTorch, OpenAI SDK, Whisper, and other libraries for Modules 3-4.

**Time**: 5-10 minutes (PyTorch is large)

## Step 8: Verify Complete Setup

Run the verification script:

```bash
cd ~/physical-ai-code-examples/src/physical_ai_utils/scripts
chmod +x verify_setup.sh
./verify_setup.sh
```

**Expected output**:
```
✅ Ubuntu 22.04 LTS detected
✅ ROS 2 Humble installed (version 0.18.x)
✅ Python 3.10+ detected (3.10.12)
✅ NVIDIA GPU detected (GeForce RTX 3060)
✅ Gazebo 11.x.x installed
✅ colcon build tool available

🎉 All checks passed! Your environment is ready for Physical AI development.
```

**If warnings about GPU**: Normal if you don't have NVIDIA GPU. You can still complete Modules 1-2.

## Step 9: Test ROS 2 Hello World

Create a test workspace:

```bash
mkdir -p ~/ros2_test_ws/src
cd ~/ros2_test_ws
colcon build
source install/setup.bash
```

Run demo nodes (open two terminals):

**Terminal 1** (talker):
```bash
source ~/ros2_test_ws/install/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** (listener):
```bash
source ~/ros2_test_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

**Expected**: Terminal 2 prints messages published by Terminal 1.

**Success!** ROS 2 pub/sub communication working.

## Common Pitfalls

### Issue: `ros2: command not found`
**Fix**: Forgot to source ROS 2. Run:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Gazebo crashes with "libGL error"
**Fix**: Graphics driver issue. Install mesa drivers:
```bash
sudo apt install libgl1-mesa-glx libglu1-mesa -y
```

### Issue: `nvidia-smi` shows "Failed to initialize NVML"
**Fix**: Reboot required after driver install, or driver mismatch:
```bash
sudo apt purge nvidia-* -y
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue: `colcon build` fails with "package not found"
**Fix**: Install missing dependencies with rosdep:
```bash
cd ~/ros2_test_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Summary

**Installed**:
- ✅ Ubuntu 22.04 LTS
- ✅ ROS 2 Humble with development tools
- ✅ Gazebo Classic 11
- ✅ NVIDIA drivers (optional)
- ✅ Python 3.10+ with robotics libraries
- ✅ VS Code + extensions
- ✅ Code repository cloned

**Total time**: 1-2 hours

**Verification**: All checks passed in `verify_setup.sh`

**Next**: If you're using cloud instead of local setup, see [Cloud Alternatives](/docs/getting-started/cloud-alternatives) for AWS EC2 configuration. Otherwise, proceed to [Module 1: ROS 2 Basics](/docs/module-01-ros2/index) to start building robot controllers.
