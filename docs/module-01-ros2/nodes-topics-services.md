---
id: nodes-topics-services
title: Nodes, Topics, and Services
sidebar_position: 3
description: Hands-on tutorial building publisher/subscriber nodes and service client/server patterns in ROS 2 with Python.
keywords: [ros 2 tutorial, publisher subscriber, service client server, rclpy, topics, services, callback functions]
reading_time: 15
---

# Nodes, Topics, and Services

**Theory is useless without practice.** In this tutorial, you'll write your first ROS 2 nodes, publish messages to topics, subscribe to data streams, and call services. By the end, you'll have working code demonstrating the three core communication patterns.

## Tutorial Overview

**What you'll build**:
1. **Publisher node**: Sends string messages to `/chatter` topic at 2 Hz
2. **Subscriber node**: Receives and logs messages from `/chatter`
3. **Service server**: Adds two integers on request
4. **Service client**: Calls the addition service

**Total time**: 30-45 minutes

## Prerequisites

Before starting:
- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- Basic Python knowledge (classes, functions, imports)
- Terminal open in your home directory (`cd ~`)

##Tutorial 1: Publisher/Subscriber

### Step 1: Create Workspace

```bash
mkdir -p ~/ros2_tutorial_ws/src
cd ~/ros2_tutorial_ws/src
```

**What this does**: Creates a ROS 2 workspace structure. The `src/` directory will hold your packages.

### Step 2: Create Python Package

```bash
ros2 pkg create --build-type ament_python pub_sub_tutorial
cd pub_sub_tutorial/pub_sub_tutorial
```

**What this does**:
- `ros2 pkg create` generates boilerplate package files
- `--build-type ament_python` specifies Python package (vs C++)
- Creates `pub_sub_tutorial/pub_sub_tutorial/` nested directory (package name / Python module)

**Verify structure**:
```bash
ls ~/ros2_tutorial_ws/src/pub_sub_tutorial/
```

**Expected output**: `package.xml`, `setup.py`, `pub_sub_tutorial/` directory

### Step 3: Write Publisher Node

Create `~/ros2_tutorial_ws/src/pub_sub_tutorial/pub_sub_tutorial/talker_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')  # Node name
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz (every 0.5s)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS 2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `Node` base class: All ROS 2 nodes inherit from this
- `create_publisher(MessageType, topic_name, queue_size)`: Creates publisher
- `create_timer(period_sec, callback)`: Calls `timer_callback` every 0.5s
- `spin(node)`: Keeps node running, processing callbacks

### Step 4: Write Subscriber Node

Create `~/ros2_tutorial_ws/src/pub_sub_tutorial/pub_sub_tutorial/listener_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_subscription(MessageType, topic_name, callback, queue_size)`: Creates subscriber
- `listener_callback(msg)`: Called automatically when message arrives
- `msg.data`: Access message field (String has one field: `data`)

### Step 5: Configure Package

Edit `~/ros2_tutorial_ws/src/pub_sub_tutorial/setup.py`:

Find the `entry_points` section and add:

```python
entry_points={
    'console_scripts': [
        'talker = pub_sub_tutorial.talker_node:main',
        'listener = pub_sub_tutorial.listener_node:main',
    ],
},
```

**What this does**: Tells `colcon` to create executable scripts named `talker` and `listener`.

### Step 6: Build Workspace

```bash
cd ~/ros2_tutorial_ws
colcon build
source install/setup.bash
```

**Expected output**:
```
Starting >>> pub_sub_tutorial
Finished <<< pub_sub_tutorial [0.50s]

Summary: 1 package finished [0.52s]
```

**If errors**: Check Python syntax, ensure `setup.py` has correct entry points.

### Step 7: Run Publisher and Subscriber

**Terminal 1** (talker):
```bash
source ~/ros2_tutorial_ws/install/setup.bash
ros2 run pub_sub_tutorial talker
```

**Expected output**:
```
[INFO] [talker]: Publishing: "Hello from ROS 2! Count: 0"
[INFO] [talker]: Publishing: "Hello from ROS 2! Count: 1"
[INFO] [talker]: Publishing: "Hello from ROS 2! Count: 2"
...
```

**Terminal 2** (listener):
```bash
source ~/ros2_tutorial_ws/install/setup.bash
ros2 run pub_sub_tutorial listener
```

**Expected output**:
```
[INFO] [listener]: I heard: "Hello from ROS 2! Count: 0"
[INFO] [listener]: I heard: "Hello from ROS 2! Count: 1"
[INFO] [listener]: I heard: "Hello from ROS 2! Count: 2"
...
```

**Success!** Publisher sends messages, subscriber receives them.

### Step 8: Inspect Topics

**Terminal 3**:
```bash
ros2 topic list
```

**Output**: `/chatter` appears in list

```bash
ros2 topic echo /chatter
```

**Output**: Live stream of messages (same as listener sees)

```bash
ros2 topic hz /chatter
```

**Output**: `average rate: 2.000` (confirms 2 Hz publishing rate)

## Tutorial 2: Service Client/Server

### Step 1: Create Service Package

```bash
cd ~/ros2_tutorial_ws/src
ros2 pkg create --build-type ament_python service_tutorial
cd service_tutorial/service_tutorial
```

### Step 2: Write Service Server

Create `~/ros2_tutorial_ws/src/service_tutorial/service_tutorial/add_server.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Add Two Ints Server ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_service(ServiceType, service_name, callback)`: Creates service server
- `add_callback(request, response)`: Computes and returns response
- `request.a`, `request.b`: Input fields from client
- `response.sum`: Output field sent back to client

### Step 3: Write Service Client

Create `~/ros2_tutorial_ws/src/service_tutorial/service_tutorial/add_client.py`:

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b

        future = self.cli.call_async(req)
        return future


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: ros2 run service_tutorial add_client <num1> <num2>')
        return

    node = AddTwoIntsClient()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
        node.get_logger().info(f'Result: {sys.argv[1]} + {sys.argv[2]} = {result.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key concepts**:
- `create_client(ServiceType, service_name)`: Creates service client
- `wait_for_service()`: Blocks until server is available
- `call_async(request)`: Sends request, returns `Future` object
- `spin_until_future_complete()`: Waits for response

### Step 4: Configure Package

Edit `~/ros2_tutorial_ws/src/service_tutorial/setup.py`:

```python
entry_points={
    'console_scripts': [
        'add_server = service_tutorial.add_server:main',
        'add_client = service_tutorial.add_client:main',
    ],
},
```

### Step 5: Build and Run

```bash
cd ~/ros2_tutorial_ws
colcon build --packages-select service_tutorial
source install/setup.bash
```

**Terminal 1** (server):
```bash
ros2 run service_tutorial add_server
```

**Output**: `[INFO] [add_two_ints_server]: Add Two Ints Server ready`

**Terminal 2** (client):
```bash
ros2 run service_tutorial add_client 10 20
```

**Expected output**:
```
[INFO] [add_two_ints_client]: Result: 10 + 20 = 30
```

**In server terminal**:
```
[INFO] [add_two_ints_server]: Request: 10 + 20 = 30
```

### Step 6: Inspect Services

```bash
ros2 service list
```

**Output**: `/add_two_ints` appears

```bash
ros2 service type /add_two_ints
```

**Output**: `example_interfaces/srv/AddTwoInts`

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

**Output**: `sum: 8`

## Common Pitfalls

### Issue: `ModuleNotFoundError: No module named 'pub_sub_tutorial'`
**Fix**: Forgot to build or source workspace:
```bash
cd ~/ros2_tutorial_ws
colcon build
source install/setup.bash
```

### Issue: Publisher runs but subscriber sees nothing
**Fix**: Check topic names match exactly (including `/` prefix):
```bash
ros2 topic list  # Verify /chatter exists
ros2 topic echo /chatter  # Verify messages flowing
```

### Issue: Service client says "Waiting for service..." forever
**Fix**: Server not running. Start server first, then client.

### Issue: `colcon build` fails with Python syntax error
**Fix**: Check indentation (Python requires 4 spaces, not tabs). Verify all colons (`:`) after `def` and `class`.

## Verification Checklist

- [ ] Publisher sends messages at 2 Hz (`ros2 topic hz /chatter` shows ~2.000)
- [ ] Subscriber receives and logs all messages
- [ ] Service server responds to client requests with correct sum
- [ ] `ros2 topic list` shows `/chatter`
- [ ] `ros2 service list` shows `/add_two_ints`
- [ ] No errors in any terminal

**All checks passed?** You've successfully built functional ROS 2 nodes!

## Summary

- **Publisher**: `create_publisher(msg_type, topic, queue_size)` + `timer_callback()` + `publish(msg)`
- **Subscriber**: `create_subscription(msg_type, topic, callback, queue_size)` + callback receives `msg`
- **Service Server**: `create_service(srv_type, name, callback)` + callback returns `response`
- **Service Client**: `create_client(srv_type, name)` + `call_async(request)` + `spin_until_future_complete()`
- **Build process**: `colcon build` + `source install/setup.bash`
- **Debugging tools**: `ros2 topic list/echo/hz`, `ros2 service list/call`

**Next**: Learn how to organize code into proper packages with [Building Packages](/docs/module-01-ros2/building-packages).
