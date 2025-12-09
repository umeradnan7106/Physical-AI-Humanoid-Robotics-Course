---
id: cloud-alternatives
title: Cloud Alternatives (AWS EC2)
sidebar_position: 4
description: Step-by-step guide to setting up a GPU-powered Physical AI development environment on AWS EC2 g5.2xlarge.
keywords: [aws ec2, cloud robotics, nvidia a10g, remote desktop, vnc, cloud development, gpu instance]
reading_time: 9
---

# Cloud Alternatives (AWS EC2)

**Don't want to install Ubuntu locally?** Use AWS EC2 to get a pre-configured development environment with a powerful NVIDIA A10G GPU for ~$1.21/hour.

## When to Use Cloud

**Choose cloud if**:
- You're on Windows/macOS and don't want to dual-boot
- You don't own an NVIDIA GPU (or yours has `<8 GB` VRAM)
- You want to try the course before committing to local setup
- You need temporary access to powerful hardware

**Choose local if**:
- You already use Linux or are willing to dual-boot
- You have NVIDIA GPU with 8+ GB VRAM
- You prefer one-time costs over recurring charges
- You want offline access

## Cost Breakdown

**Recommended instance**: `g5.2xlarge`
- **Hourly rate**: ~$1.21/hour (us-east-1 region, on-demand pricing)
- **Course usage** (8 hours/week × 13 weeks): ~$125
- **Storage** (100 GB EBS): ~$10/month × 3 months = ~$30
- **Total estimated cost**: ~$155-$205 (depends on actual usage)

**Cost-saving tips**:
1. Stop instance when not in use (storage costs continue, compute stops)
2. Use Spot Instances for 60-70% discount (risk: can be terminated anytime)
3. Delete instance after completing modules (keep snapshots for later)

## Step 1: Create AWS Account

1. Go to [aws.amazon.com](https://aws.amazon.com)
2. Click "Create an AWS Account"
3. Enter email, password, account name
4. Add payment method (credit card required even for free tier)
5. Verify phone number
6. Choose "Basic Support - Free" plan

**Note**: New AWS accounts get 12 months of limited free tier (not applicable to g5.2xlarge GPU instances, which are always paid).

## Step 2: Launch EC2 Instance

### 2.1 Navigate to EC2

1. Sign in to AWS Console
2. Search for "EC2" in top search bar
3. Click "EC2" (Virtual Servers in the Cloud)
4. Click "Launch Instance" (orange button)

### 2.2 Configure Instance

**Name**: `physical-ai-dev` (or any name you prefer)

**Application and OS Images (AMI)**:
1. Click "Browse more AMIs"
2. Search "Ubuntu 22.04 LTS"
3. Select **Ubuntu Server 22.04 LTS (HVM), SSD Volume Type**
4. Architecture: **64-bit (x86)**

**Instance type**:
1. Click instance type dropdown
2. Filter by "GPU instances"
3. Select **g5.2xlarge**
   - 8 vCPUs (AMD EPYC 7R32)
   - 32 GB RAM
   - 1x NVIDIA A10G GPU (24 GB VRAM)

**Key pair (login)**:
1. Click "Create new key pair"
2. Name: `physical-ai-key`
3. Type: **RSA**
4. Format: **.pem** (for macOS/Linux) or **.ppk** (for PuTTY on Windows)
5. Click "Create key pair" (downloads file)
6. **IMPORTANT**: Save this file securely. You'll need it to connect via SSH.

**Network settings**:
1. Click "Edit" next to Network settings
2. **Auto-assign public IP**: Enable
3. **Security group**: Create new
4. Add rules:
   - **SSH** (port 22): Source "My IP" (for SSH access)
   - **Custom TCP** (port 5900): Source "My IP" (for VNC remote desktop)
5. Security group name: `physical-ai-sg`

**Storage**:
1. Change size from default (8 GB) to **100 GB**
2. Volume type: **gp3** (General Purpose SSD)
3. Delete on termination: **Yes** (recommended; can disable if you want persistent storage)

**Advanced details** (optional):
- Scroll down to "User data" (optional)
- Can paste bash script to auto-install tools on first boot (we'll do manual install for learning)

### 2.3 Launch

1. Review summary on right sidebar
2. Click "Launch instance" (orange button)
3. Wait 1-2 minutes for instance to start
4. Status shows "Running" when ready

## Step 3: Connect to Instance

### 3.1 Get Instance IP Address

1. Go to EC2 Dashboard
2. Click "Instances (running)"
3. Select your instance
4. Copy **Public IPv4 address** (e.g., `54.123.45.67`)

### 3.2 Connect via SSH

**On macOS/Linux**:

```bash
# Set permissions on key file (first time only)
chmod 400 ~/Downloads/physical-ai-key.pem

# Connect to instance
ssh -i ~/Downloads/physical-ai-key.pem ubuntu@<YOUR_PUBLIC_IP>
```

Replace `<YOUR_PUBLIC_IP>` with your instance's IP address.

**On Windows**:
1. Download and install [PuTTY](https://www.putty.org/)
2. Use PuTTYgen to convert `.pem` to `.ppk` (if you downloaded `.pem`)
3. Open PuTTY:
   - Host: `ubuntu@<YOUR_PUBLIC_IP>`
   - Port: 22
   - Connection → SSH → Auth → Browse → Select `.ppk` file
4. Click "Open"

**Expected**: Terminal prompt `ubuntu@ip-xxx-xxx-xxx-xxx:~$`

## Step 4: Install Development Environment

Run the same installation steps from [Environment Setup](/docs/getting-started/environment-setup):

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble (all steps from Environment Setup)
# ... (follow Step 2 from Environment Setup chapter)

# Install Gazebo
sudo apt install gazebo ros-humble-gazebo-ros-pkgs -y

# Install NVIDIA drivers (already pre-installed on g5 instances)
nvidia-smi  # Verify GPU

# Install Python dependencies
sudo apt install python3-pip -y
pip3 install numpy opencv-python matplotlib scipy transforms3d

# Clone code repository
cd ~
git clone https://github.com/umeradnan7106/physical-ai-code-examples.git
cd physical-ai-code-examples
pip3 install -r requirements.txt
```

**Time**: 30-40 minutes (faster than local due to AWS bandwidth)

## Step 5: Set Up Remote Desktop (VNC)

SSH is great for terminal work, but Gazebo and Isaac Sim need a GUI.

### 5.1 Install Desktop Environment

```bash
# Install lightweight desktop (XFCE)
sudo apt install xfce4 xfce4-goodies -y

# Install VNC server
sudo apt install tightvncserver -y
```

### 5.2 Configure VNC

```bash
# Start VNC server (will ask for password)
vncserver

# Enter password when prompted (8 characters max)
# Confirm password
```

**Note**: Remember this password; you'll need it to connect from your laptop.

### 5.3 Configure VNC to Use XFCE

```bash
# Kill VNC server
vncserver -kill :1

# Backup default config
mv ~/.vnc/xstartup ~/.vnc/xstartup.bak

# Create new config
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
xrdb $HOME/.Xresources
startxfce4 &
EOF

# Make executable
chmod +x ~/.vnc/xstartup

# Restart VNC
vncserver -geometry 1920x1080 -depth 24
```

**Expected output**: `New 'X' desktop is ip-xxx:1`

### 5.4 Connect from Your Laptop

**On macOS**:
1. Open Finder → Go → Connect to Server (Cmd+K)
2. Enter: `vnc://<YOUR_PUBLIC_IP>:5901`
3. Enter VNC password you set earlier

**On Windows**:
1. Download [TightVNC Viewer](https://www.tightvnc.com/download.php) or [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)
2. Connect to: `<YOUR_PUBLIC_IP>:5901`
3. Enter VNC password

**On Linux**:
```bash
vncviewer <YOUR_PUBLIC_IP>:5901
```

**Expected**: XFCE desktop opens in new window.

## Step 6: Verify GPU in Cloud

Open terminal in VNC desktop:

```bash
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.xx.xx    Driver Version: 525.xx.xx    CUDA Version: 12.x  |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
|   0  NVIDIA A10G         On   | 00000000:00:1E.0 Off |                    0 |
|  0%   32C    P8    17W / 300W |      0MiB / 23028MiB |      0%      Default |
+-----------------------------------------------------------------------------+
```

**Your GPU**: NVIDIA A10G with 24 GB VRAM (more powerful than RTX 3060!)

## Step 7: Test Gazebo with GUI

```bash
source /opt/ros/humble/setup.bash
gazebo
```

**Expected**: Gazebo window opens in VNC desktop showing empty world.

**Success!** Cloud environment is fully configured.

## Managing Your Instance

### Start/Stop (to Save Money)

**Stop instance when not in use**:
1. EC2 Dashboard → Instances
2. Select instance → Instance state → Stop instance
3. **Charges stop** for compute (still pay for storage ~$10/month)

**Restart**:
1. Instance state → Start instance
2. **Note**: Public IP changes each time you stop/start (use Elastic IP to keep same IP)

**Terminate** (delete completely):
1. Instance state → Terminate instance
2. All data deleted (unless you created snapshot)

### Create Snapshot (Backup)

Before terminating:
1. EC2 Dashboard → Instances → Select instance
2. Actions → Image and templates → Create image
3. Name: `physical-ai-snapshot-week-5`
4. Click "Create image"
5. Later: Launch new instance from this snapshot (AMIs section)

## Cost Monitoring

1. AWS Console top-right → Your name → Billing Dashboard
2. View "Month-to-Date Spend"
3. Set up billing alerts (recommended):
   - CloudWatch → Alarms → Billing → Create alarm
   - Alert when charges exceed $50

## Troubleshooting

### Issue: Can't connect via SSH
**Fix**: Security group doesn't allow SSH from your IP. Edit security group, add SSH rule with "My IP".

### Issue: VNC connection refused
**Fix**: VNC server not running. SSH into instance, run `vncserver -geometry 1920x1080`.

### Issue: Gazebo crashes immediately
**Fix**: VNC not using GPU. Install VirtualGL for GPU acceleration (advanced; see [Troubleshooting Guide](/docs/appendices/troubleshooting#vnc-gpu-issues)).

### Issue: Instance costs more than expected
**Fix**: Forgot to stop instance. Always stop when done (don't just close VNC).

## Summary

- **Instance**: AWS EC2 g5.2xlarge (~$1.21/hour)
- **GPU**: NVIDIA A10G (24 GB VRAM)
- **Cost**: ~$155-$205 for full course (8 hours/week × 13 weeks)
- **Access**: SSH (terminal) + VNC (GUI)
- **Setup time**: 30-40 minutes
- **Advantages**: No local GPU needed, works on Windows/macOS, powerful hardware
- **Disadvantages**: Recurring costs, requires internet, latency in GUI

**Next**: Proceed to [Module 1: ROS 2 Basics](/docs/module-01-ros2/index) to start learning robot programming.
