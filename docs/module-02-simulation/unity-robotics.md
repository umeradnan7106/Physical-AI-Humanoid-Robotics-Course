---
id: unity-robotics
title: Unity Robotics Integration
sidebar_position: 5
description: Connect Unity to ROS 2 for visualization and VR/AR applications.
keywords: [unity ros2, ros-tcp-connector, urdf importer, unity robotics hub]
---

# Unity Robotics Integration

## Setup Unity

1. **Install Unity Hub** and Unity 2021.3 LTS
2. **Create new project** (3D template)
3. **Install packages** via Package Manager:
   - ROS-TCP-Connector
   - URDF Importer

**Add packages**: Window → Package Manager → "+" → Add package from git URL:
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

## Configure ROS-TCP Endpoint

**In Unity**: Robotics → ROS Settings
- ROS IP Address: `127.0.0.1` (localhost)
- ROS Port: `10000`
- Protocol: `ROS 2`

**Launch ROS endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Import URDF

1. **Copy URDF** to Unity project: `Assets/URDF/humanoid.urdf`
2. **Import**: Robotics → Import Robot from URDF
3. **Select file**: Choose `humanoid.urdf`
4. **Settings**:
   - Axis Type: Z-Axis
   - Mesh Decomposer: VHACD
5. **Import**

Robot appears as GameObject with ArticulationBody components.

## Publish Unity Data to ROS 2

**C# script** (attach to robot):

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStatePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/joint_states";
    public float publishRate = 50f;

    private float timer = 0f;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            PublishJointStates();
            timer = 0f;
        }
    }

    void PublishJointStates()
    {
        JointStateMsg msg = new JointStateMsg
        {
            name = new string[joints.Length],
            position = new double[joints.Length]
        };

        for (int i = 0; i < joints.Length; i++)
        {
            msg.name[i] = joints[i].name;
            msg.position[i] = joints[i].jointPosition[0];
        }

        ros.Publish(topicName, msg);
    }
}
```

## Subscribe to ROS 2 Commands

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class TwistSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("/cmd_vel", CommandCallback);
    }

    void CommandCallback(TwistMsg msg)
    {
        // Move robot based on msg.linear.x, msg.angular.z
        transform.Translate(Vector3.forward * (float)msg.linear.x * Time.deltaTime);
    }
}
```

## Verify Connection

**In Unity console**, you should see:
```
[ROS] Connected to ROS 2 at 127.0.0.1:10000
```

**Check ROS 2 topics**:
```bash
ros2 topic list  # Should show /joint_states
ros2 topic echo /joint_states
```

## Use Cases

- **Visualization**: Prettier graphics than Gazebo/RViz
- **VR/AR**: Immersive robot teleoperation
- **Digital Twins**: Real-time mirroring of physical robots
- **Training Environments**: RL with Unity ML-Agents

**Next**: [Module Project](/docs/module-02-simulation/project-simulation)
