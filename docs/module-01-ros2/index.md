---
id: index
title: Module 1 - ROS 2 Fundamentals
sidebar_label: Overview
sidebar_position: 1
description: Master ROS 2 Humble by building the robotic nervous system - nodes, topics, services, launch files, and URDF models for humanoid robots.
keywords: [ros 2 humble, nodes topics services, publisher subscriber, urdf, launch files, colcon build, ament python]
reading_time: 7
---

# Module 1: The Robotic Nervous System - ROS 2

**ROS 2 is the nervous system of modern robots.** Just as your brain communicates with muscles through neurons, robots use ROS 2 to coordinate sensors, motors, and decision-making algorithms. In this module, you'll build the communication infrastructure that makes intelligent robots possible.

## What You'll Learn

By the end of this module (Weeks 3-5), you'll be able to:

1. **Understand ROS 2 Architecture**: Nodes, topics, services, actions, and how data flows through a robotic system
2. **Build Custom Packages**: Create your own ROS 2 packages using `ament_python` and `colcon` build tools
3. **Implement Communication Patterns**: Publisher/subscriber for sensor data, service calls for on-demand computation
4. **Write Launch Files**: Orchestrate multiple nodes with parameters and configurations
5. **Create URDF Models**: Define humanoid robot structure with joints, links, and kinematics for RViz visualization
6. **Develop Teleoperation**: Control simulated robots with keyboard input

**Hands-on projects**: You'll build 4 working ROS 2 packages:
- Publisher/subscriber system for robot communication
- Service client/server for computation requests
- URDF-based humanoid model with 10 degrees of freedom
- Keyboard teleoperation controller

## Why ROS 2?

**ROS (Robot Operating System)** is not an operating system—it's a middleware framework that provides:

- **Standardized communication**: Sensors and motors speak the same language
- **Modular architecture**: Swap out components without rewriting entire systems
- **Massive ecosystem**: 3,000+ open-source packages for navigation, perception, manipulation
- **Industry adoption**: Used by Tesla (Optimus), NASA (Mars rovers), Boston Dynamics, and thousands of research labs

**ROS 2 vs ROS 1**:
- Real-time capable (critical for control loops)
- Built on DDS (Data Distribution Service) for reliable communication
- Native support for multiple robots (swarm coordination)
- Production-ready security (authentication, encryption)
- Windows/macOS support (though we use Ubuntu for best compatibility)

**This book uses ROS 2 Humble** (LTS release, supported until 2027).

## Module Structure

```mermaid
graph LR
    A[Week 3: Architecture] --> B[ROS 2 Core Concepts]
    B --> C[Nodes & Topics]
    C --> D[Week 4: Development]
    D --> E[Building Packages]
    E --> F[Services & Launch Files]
    F --> G[Week 5: Integration]
    G --> H[URDF Modeling]
    H --> I[Teleoperation Project]
    I --> J[Module Assessment]
```

### Week 3: ROS 2 Architecture & Communication
**Learning outcomes**: Understand node graph, topic-based pub/sub, service request/response patterns

**Chapters**:
1. [ROS 2 Architecture](/docs/module-01-ros2/ros2-architecture) - DDS layer, node graph, QoS policies
2. [Nodes, Topics, Services](/docs/module-01-ros2/nodes-topics-services) - Hands-on publisher/subscriber and service tutorials

**Project**: Create talker/listener nodes communicating via `/chatter` topic

### Week 4: Package Development & Launch Files
**Learning outcomes**: Build custom ROS 2 packages, manage dependencies, orchestrate multi-node systems

**Chapters**:
3. [Building Packages](/docs/module-01-ros2/building-packages) - `ament_python`, `setup.py`, `package.xml`, `colcon build`
4. [Launch Files](/docs/module-01-ros2/launch-files) - Python launch API, parameters, namespaces, composable nodes

**Project**: Service client/server package with launch file automation

### Week 5: URDF & Robot Modeling
**Learning outcomes**: Define robot structure in URDF, visualize in RViz, implement teleoperation

**Chapters**:
5. [URDF for Humanoids](/docs/module-01-ros2/urdf-humanoids) - Links, joints, xacro macros, collision geometry
6. [Module Project: ROS 2 Package](/docs/module-01-ros2/project-ros2-package) - Build complete humanoid controller with teleoperation

**Capstone Project**: 10-DOF humanoid URDF + keyboard teleop node

## Prerequisites

Before starting this module, you should have:

- ✅ Ubuntu 22.04 LTS with ROS 2 Humble installed (from [Getting Started](/docs/getting-started/environment-setup))
- ✅ Basic Python knowledge (functions, classes, imports)
- ✅ Terminal familiarity (cd, ls, running bash commands)
- ✅ Successfully run `verify_setup.sh` with all checks passing

**No prior robotics experience required.** We explain every ROS 2 concept from scratch.

## Tools You'll Use

- **ROS 2 CLI**: `ros2 run`, `ros2 topic`, `ros2 service`, `ros2 node`
- **colcon**: Build system for ROS 2 workspaces
- **RViz**: 3D visualization tool for robot models and sensor data
- **rqt_graph**: Visualize node communication graph
- **VS Code**: Recommended IDE with ROS extension

## Time Commitment

**Total**: 3 weeks (Weeks 3-5 of the course)

**Estimated hours per week**:
- Reading chapters: 2-3 hours
- Hands-on tutorials: 4-5 hours
- Debugging and experimentation: 2-3 hours

**Total time**: 24-33 hours over 3 weeks

**Flexible pacing**: All content is self-paced. Complete faster if you have more time, or spread it over more weeks.

## What You'll Build

### Project 1: Publisher/Subscriber System
A simple communication demo where one node publishes messages and another subscribes.

**Skills**: `rclpy`, topic creation, message types, callback functions

### Project 2: Service Client/Server
A computation service that adds two integers on request.

**Skills**: Service interfaces, synchronous vs asynchronous calls, error handling

### Project 3: Humanoid URDF Model
A 10-DOF robot model with torso, arms, legs, and head.

**Skills**: URDF syntax, joint types (revolute, fixed), xacro macros, RViz visualization

### Project 4: Teleoperation Controller
Keyboard input controls robot joint positions.

**Skills**: User input handling, publishing joint commands, real-time control

**Portfolio value**: By the end of this module, you'll have a GitHub repository with 4 complete ROS 2 packages demonstrating core robotics skills that employers actively seek.

## Assessment & Next Steps

**Self-assessment checklist** (end of module):
- [ ] I can explain the difference between topics, services, and actions
- [ ] I can create a custom ROS 2 package with correct dependency management
- [ ] I can write a launch file to start multiple nodes with parameters
- [ ] I can create a URDF file and visualize it in RViz
- [ ] I successfully completed all 4 projects with working code

**If you checked all boxes**: Proceed to [Module 2: Simulation](/docs/module-02-simulation/index)

**If you struggled**: Review the [Troubleshooting Guide](/docs/appendices/troubleshooting#module-01-issues) or revisit specific chapters

## Common Questions

**Q: Do I need a physical robot?**
A: No. Everything in this module runs in simulation. You'll visualize robots in RViz and control them programmatically.

**Q: I'm stuck on a tutorial. Where do I get help?**
A: Check the Troubleshooting Guide, review the code examples in the GitHub repository, or open an issue for community support.

**Q: Can I skip this module if I already know ROS 1?**
A: ROS 2 has significant architectural differences (DDS, launch file syntax, build system). We recommend at least skimming chapters 1-2 to understand the migration.

**Q: How is ROS 2 used in real humanoid robots?**
A: Tesla Optimus runs ROS 2 for motor control and sensor fusion. NASA's Valkyrie uses it for task planning. Academic labs worldwide build on ROS 2 for research prototypes.

## Ready to Begin?

Start with [ROS 2 Architecture](/docs/module-01-ros2/ros2-architecture) to understand the foundational concepts, then progress through hands-on tutorials building real robot controllers.

**Remember**: Robotics is learned by doing. Type every command, run every example, break things and fix them. That's how expertise develops.

---

**Next**: Dive into [ROS 2 Architecture](/docs/module-01-ros2/ros2-architecture) to understand how nodes, topics, and services form the robotic nervous system.
