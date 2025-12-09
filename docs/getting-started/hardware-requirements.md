---
id: hardware-requirements
title: Hardware Requirements
sidebar_position: 2
description: Detailed hardware specifications for local Physical AI development, from minimum requirements to optimal configurations.
keywords: [hardware requirements, ubuntu system requirements, nvidia gpu, robotics development, local setup, system specs]
reading_time: 6
---

# Hardware Requirements

**Good news: You don't need a robotics lab to learn Physical AI.** This chapter covers exactly what hardware you need for local development, with budget-friendly alternatives and cloud options for expensive components.

## Minimum vs. Recommended Specs

### Minimum Requirements (Modules 1-2)

**Works for**: ROS 2 fundamentals + Gazebo Classic simulation

| Component | Minimum Spec | Notes |
|-----------|--------------|-------|
| **CPU** | Intel Core i5-8th gen / AMD Ryzen 5 3600 | 4 cores, 2.5 GHz+ |
| **RAM** | 8 GB DDR4 | Gazebo simulations may lag with `<8 GB` |
| **Storage** | 50 GB free space (SSD) | HDD works but ROS 2 builds are 5x slower |
| **GPU** | None required | CPU-only rendering for Gazebo |
| **OS** | Ubuntu 22.04 LTS | **Strict requirement** (ROS 2 Humble compatibility) |
| **Network** | Broadband internet | For initial package downloads (~5 GB) |

**What you can do**:
- Learn ROS 2 nodes, topics, services, actions
- Build and simulate wheeled robots in Gazebo
- Implement basic path planning and navigation
- Complete Modules 1-2 entirely

**Limitations**:
- Gazebo rendering may be slow (10-15 FPS)
- Cannot run Isaac Sim (Module 3 requires GPU)
- Local LLM inference not practical (use cloud APIs)

**Example budget build**: Refurbished Dell OptiPlex 7050 (~$300) + Ubuntu 22.04

### Recommended Specs (All Modules)

**Works for**: Full course including Isaac Sim (Module 3) + local LLM inference

| Component | Recommended Spec | Notes |
|-----------|------------------|-------|
| **CPU** | Intel Core i7-12th gen / AMD Ryzen 7 5800X | 8+ cores for parallel simulation |
| **RAM** | 16 GB DDR4 (32 GB ideal) | Isaac Sim uses 10-12 GB VRAM + 8 GB system RAM |
| **Storage** | 100 GB free space (NVMe SSD) | Isaac Sim installation: 30 GB, cache: 20 GB+ |
| **GPU** | NVIDIA RTX 3060 (12 GB VRAM) or better | RTX 4070, RTX 4080, or A5000 for heavy workloads |
| **OS** | Ubuntu 22.04 LTS | Fresh install recommended (not WSL2) |
| **Network** | Broadband internet | For Isaac Sim downloads (~40 GB) |

**What you can do**:
- Run photorealistic Isaac Sim environments (30-60 FPS)
- Train vision models locally (YOLO, Mask R-CNN)
- Run local LLMs (Llama 3, Mistral) for VLA tasks
- Complete all modules without cloud costs

**Example build**: Custom desktop with RTX 4070 (~$1,200) or gaming laptop (ASUS ROG Strix ~$1,500)

## Deep Dive: Components

### CPU

**Why it matters**: ROS 2 builds, Gazebo physics simulations, and parallel sensor processing.

**Minimum**: 4 cores (Intel i5-8th gen / AMD Ryzen 5 3600)
**Recommended**: 8+ cores (Intel i7-12th gen / AMD Ryzen 7 5800X)

**Real-world impact**:
- Building ROS 2 workspace with 10 packages: 4-core (5 minutes) vs. 8-core (2 minutes)
- Gazebo with 5 robots: 4-core (12 FPS) vs. 8-core (30 FPS)

**Budget tip**: Older server CPUs (Intel Xeon E5-2680 v2) offer 10 cores for ~$50 used.

### RAM

**Why it matters**: Gazebo loads entire simulation in memory. Isaac Sim caches textures and meshes. Multiple ROS 2 nodes run concurrently.

**Minimum**: 8 GB (tight; close browser tabs during Gazebo)
**Recommended**: 16 GB (comfortable for Modules 1-2)
**Optimal**: 32 GB (Isaac Sim + Chrome + VS Code simultaneously)

**Real-world impact**:
- Gazebo warehouse simulation (20 objects): 6 GB RAM usage
- Isaac Sim (medium scene): 10-12 GB GPU VRAM + 8 GB system RAM
- Running local Llama 3 (8B): 8-10 GB RAM

**Budget tip**: RAM is cheap (~$30 for 16 GB DDR4). Upgrade this first if budget is tight.

### Storage

**Why it matters**: ROS 2 packages, Isaac Sim installation, Docker images, build artifacts.

**Minimum**: 50 GB free (SSD strongly recommended)
**Recommended**: 100 GB free (NVMe SSD for faster builds)

**Space breakdown**:
- Ubuntu 22.04: 10 GB
- ROS 2 Humble + dependencies: 5 GB
- Gazebo + models: 3 GB
- Isaac Sim 2023.1.1: 30 GB
- Docker images (if used): 10-20 GB
- ROS 2 workspace builds: 5-10 GB

**Why SSD?**: ROS 2 `colcon build` reads/writes thousands of small files. SSD is 5-10x faster than HDD.

**Budget tip**: Use smaller SSD for OS/tools (256 GB ~$30), larger HDD for data storage (1 TB ~$40).

### GPU (Optional but Impactful)

**Why it matters**: Isaac Sim requires NVIDIA GPU with RTX technology (ray tracing cores). Local AI inference (YOLO, LLMs) runs 10-50x faster on GPU.

**GPU requirements by module**:

| Module | GPU Needed? | Minimum VRAM | Recommended GPU | Alternatives |
|--------|-------------|--------------|-----------------|--------------|
| Module 1 (ROS 2) | No | N/A | N/A | CPU-only works |
| Module 2 (Gazebo) | No | N/A | N/A | CPU rendering OK |
| Module 3 (Isaac Sim) | **Yes** | 8 GB | RTX 3060 (12 GB) | AWS g5.2xlarge cloud instance |
| Module 4 (VLA) | Recommended | 12 GB | RTX 4070 (12 GB) | Use OpenAI/Anthropic APIs |

**Supported GPUs** (NVIDIA only; AMD/Intel not compatible with Isaac Sim):
- **Budget**: RTX 3060 (12 GB VRAM, ~$300 used)
- **Mid-range**: RTX 4070 (12 GB VRAM, ~$550)
- **High-end**: RTX 4080 (16 GB VRAM, ~$1,000)
- **Workstation**: RTX A5000 (24 GB VRAM, ~$2,000)

**VRAM recommendations**:
- 8 GB: Isaac Sim small scenes (single robot)
- 12 GB: Isaac Sim medium scenes (warehouse with 10-20 objects)
- 16 GB+: Isaac Sim large scenes + local LLM inference

**What if I don't have NVIDIA GPU?**
- Modules 1-2: Proceed with CPU-only (no impact)
- Module 3: Use AWS EC2 g5.2xlarge (~$200 for course duration)
- Module 4: Use cloud API calls (OpenAI $20/month usage)

### Operating System: Ubuntu 22.04 LTS (Non-Negotiable)

**Why Ubuntu 22.04 specifically?**
- ROS 2 Humble officially supports Ubuntu 22.04 (Tier 1 platform)
- NVIDIA drivers work reliably on Ubuntu
- Isaac Sim validated on Ubuntu 22.04
- Most robotics labs worldwide use Ubuntu (industry standard)

**Why not Windows/macOS?**
- WSL2 (Windows Subsystem for Linux): Doesn't support GPU passthrough for Gazebo/Isaac Sim reliably
- macOS: No NVIDIA GPU support, ROS 2 support limited
- Docker on Windows/macOS: GUI performance is poor

**Installation options**:
1. **Dual-boot**: Keep Windows/macOS, add Ubuntu partition (recommended for beginners)
2. **Full install**: Wipe existing OS, install Ubuntu only (best performance)
3. **Cloud**: Skip local install, use AWS EC2 (covered in [Cloud Alternatives](/docs/getting-started/cloud-alternatives))

## Peripherals

**Required**:
- Keyboard, mouse, monitor (standard equipment)

**Helpful but optional**:
- **Second monitor**: View Gazebo/Isaac Sim on one screen, code on another
- **External webcam**: Test computer vision code with real camera feed
- **USB gamepad**: Control simulated robots (Xbox/PlayStation controller)

## Budget Scenarios

### Ultra-Budget ($0 - Cloud Only)
- Use existing Windows/macOS computer
- AWS EC2 g5.2xlarge for Modules 3-4 (~$205)
- Total: **~$200**

### Minimal Local ($300-$500)
- Refurbished desktop (i5-8th gen, 8 GB RAM, SSD)
- Ubuntu 22.04 (free)
- Modules 1-2 locally, cloud for Module 3
- Total: **~$300-$400**

### Recommended Local ($1,000-$1,500)
- Gaming desktop or laptop (i7-12th gen, 16 GB RAM, RTX 4070)
- Ubuntu 22.04 dual-boot
- All modules locally (no cloud costs)
- Total: **~$1,200-$1,500**

## Can I Use What I Already Have?

**Check these**:
1. Run `lscpu` (Linux) or System Information (Windows) → Do you have 4+ cores?
2. Run `free -h` (Linux) or Task Manager (Windows) → Do you have 8+ GB RAM?
3. Check storage: Do you have 50+ GB free?
4. Check GPU: `lspci | grep -i nvidia` (Linux) or Device Manager (Windows) → NVIDIA GPU with 8+ GB VRAM?

**If yes to all**: Proceed with local setup (next chapter)
**If missing GPU**: Modules 1-2 work fine; plan cloud option for Module 3
**If `<8 GB` RAM or `<50 GB` storage**: Consider upgrading RAM (~$30) or external SSD

## Summary

- **Minimum (Modules 1-2)**: 4-core CPU, 8 GB RAM, 50 GB SSD, Ubuntu 22.04 (no GPU needed)
- **Recommended (all modules)**: 8-core CPU, 16 GB RAM, 100 GB NVMe SSD, RTX 3060+ GPU, Ubuntu 22.04
- **GPU requirement**: Optional for Modules 1-2, required for Isaac Sim (Module 3)
- **OS requirement**: Ubuntu 22.04 LTS (non-negotiable for ROS 2 Humble)
- **Budget options**: Refurbished desktops (~$300), cloud instances (~$200 for course)

**Next**: Proceed to [Environment Setup](/docs/getting-started/environment-setup) to install Ubuntu 22.04, ROS 2 Humble, and all required tools.
