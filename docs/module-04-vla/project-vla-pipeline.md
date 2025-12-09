---
id: project-vla-pipeline
title: Project - VLA Pipeline
sidebar_position: 6
description: Build end-to-end voice-controlled object retrieval system.
keywords: [vla project, voice control, object retrieval, end to end]
---

# Project: Voice-Controlled Object Retrieval

## Objective

Build complete VLA pipeline: **"Bring me the red cup from the kitchen"** → Robot navigates, grasps cup, returns.

**Success criteria**: 80%+ success rate over 10 voice command trials.

## Requirements

### 1. Environment Setup
- **Simulation**: Isaac Sim or Gazebo with apartment scene
- **Objects**: 3 colored cups (red, blue, green) on kitchen counter
- **Locations**: Kitchen, living room, table (semantic map)
- **Navigation**: Nav2 stack from Module 3
- **Microphone**: USB or laptop built-in

### 2. VLA Pipeline Components
- **Whisper**: Real-time speech transcription
- **LLM Planner**: Ollama (Llama 3.1 8B) or GPT-4
- **Action Executor**: Navigation + simulated pick/place
- **Master launch file**: Start all components

### 3. Test Commands
```
"Go to the kitchen"
"Bring me a cup from the kitchen"
"Pick up the red cup and place it on the table"
"Navigate to the living room and wait 5 seconds"
```

### 4. Deliverables
- Working demo video (2+ minutes)
- Rosbag recording
- Performance report (success rate, latency breakdown)

## Implementation

### Step 1: Master Launch File

**vla_pipeline.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Whisper STT node
        Node(
            package='whisper_ros',
            executable='whisper_node',
            output='screen'
        ),

        # LLM Planner node
        Node(
            package='llm_planner',
            executable='planner_node',
            parameters=[{
                'llm_provider': os.getenv('LLM_PROVIDER', 'ollama')
            }],
            output='screen'
        ),

        # Action Executor node
        Node(
            package='action_executor',
            executable='executor_node',
            output='screen'
        ),

        # Nav2 stack (assumes Isaac Sim/Gazebo already running)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py'],
            output='screen'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                os.getenv('HOME'),
                'vla_ws/src/vla_config/rviz/vla_pipeline.rviz'
            )]
        )
    ])
```

**Launch**:
```bash
# Terminal 1: Start Isaac Sim/Gazebo
# Terminal 2: Launch VLA pipeline
ros2 launch vla_pipeline vla_pipeline.launch.py
```

### Step 2: Semantic Map Configuration

**semantic_map.yaml**:
```yaml
locations:
  kitchen:
    position: [2.5, 3.0, 0.0]
    objects:
      - red_cup
      - blue_cup
      - green_cup

  living_room:
    position: [-1.0, 2.0, 0.0]
    objects:
      - sofa
      - lamp

  table:
    position: [1.0, 1.0, 0.0]
    objects: []

  bedroom:
    position: [5.0, -2.0, 0.0]
    objects:
      - bed

objects:
  red_cup:
    location: kitchen
    position: [2.6, 3.2, 0.8]

  blue_cup:
    location: kitchen
    position: [2.5, 3.2, 0.8]

  green_cup:
    location: kitchen
    position: [2.4, 3.2, 0.8]
```

**Load in executor_node.py**:
```python
import yaml

def __init__(self):
    # ... (previous code)

    # Load semantic map
    with open('semantic_map.yaml', 'r') as f:
        map_data = yaml.safe_load(f)
        self.semantic_map = {
            loc: tuple(data['position'][:2])
            for loc, data in map_data['locations'].items()
        }
        self.objects = map_data['objects']
```

### Step 3: Enhanced LLM Prompt

**Updated system prompt** (with object awareness):
```python
ROBOT_SYSTEM_PROMPT = """You are a robot task planner in an apartment.

Available actions:
- navigate: Move to location (params: location_name)
- pick: Grasp object (params: object_id)
- place: Release object (params: location_name)
- wait: Pause execution (params: duration_seconds)

Environment:
Locations: kitchen (has red_cup, blue_cup, green_cup), living_room, table, bedroom
Objects: red_cup, blue_cup, green_cup (all in kitchen)

Example 1:
User: "Go to the kitchen"
Output: {"steps": [{"action": "navigate", "params": {"location_name": "kitchen"}}]}

Example 2:
User: "Bring me the red cup from the kitchen"
Output: {
  "steps": [
    {"action": "navigate", "params": {"location_name": "kitchen"}},
    {"action": "pick", "params": {"object_id": "red_cup"}},
    {"action": "navigate", "params": {"location_name": "table"}},
    {"action": "place", "params": {"location_name": "table"}}
  ]
}

Output ONLY valid JSON.
"""
```

### Step 4: Testing Script

**test_vla_pipeline.py**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class VLATester(Node):
    def __init__(self):
        super().__init__('vla_tester')
        self.transcript_pub = self.create_publisher(String, '/voice/transcript', 10)
        time.sleep(2)  # Wait for subscribers

    def test_command(self, command: str):
        self.get_logger().info(f'Testing: "{command}"')
        msg = String()
        msg.data = command
        self.transcript_pub.publish(msg)
        time.sleep(10)  # Wait for execution

def main():
    rclpy.init()
    tester = VLATester()

    test_commands = [
        "Go to the kitchen",
        "Navigate to the living room and wait 3 seconds",
        "Bring me the red cup from the kitchen"
    ]

    for cmd in test_commands:
        tester.test_command(cmd)
        input("Press Enter for next test...")

# Run
ros2 run vla_tests test_vla_pipeline
```

### Step 5: Performance Monitoring

**Monitor topics**:
```bash
# Terminal 1: Watch transcripts
ros2 topic echo /voice/transcript

# Terminal 2: Watch LLM plans
ros2 topic echo /robot/action_plan

# Terminal 3: Watch Nav2 status
ros2 topic echo /navigate_to_pose/_action/status
```

**Measure latency**:
```python
import time

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        self.transcript_time = None

        self.transcript_sub = self.create_subscription(
            String, '/voice/transcript', self.transcript_callback, 10
        )
        self.plan_sub = self.create_subscription(
            String, '/robot/action_plan', self.plan_callback, 10
        )

    def transcript_callback(self, msg):
        self.transcript_time = time.time()
        self.get_logger().info(f'Transcript received: {msg.data}')

    def plan_callback(self, msg):
        if self.transcript_time:
            latency = time.time() - self.transcript_time
            self.get_logger().info(f'LLM latency: {latency:.2f}s')
```

## Evaluation Criteria

**Success checklist**:
- [ ] Whisper accurately transcribes commands (90%+ word accuracy)
- [ ] LLM generates valid JSON plans (100% valid)
- [ ] Robot navigates to correct locations (95%+ accuracy)
- [ ] Pick/place actions execute (simulated, 100%)
- [ ] End-to-end latency < 5 seconds (Whisper → LLM → Nav2 start)
- [ ] 8/10 trials complete successfully (80%+ success rate)

**Performance metrics**:
| Metric | Target | Measured |
|--------|--------|----------|
| Transcription accuracy | 90%+ | ___ |
| LLM plan validity | 100% | ___ |
| Navigation success | 95%+ | ___ |
| End-to-end latency | &lt;5s | ___s |
| Overall success rate | 80%+ | ___% |

## Extensions

1. **Multi-object retrieval**: "Bring me all cups from the kitchen"
2. **Conditional logic**: "If the table is empty, place the cup there, otherwise place on shelf"
3. **Failure recovery**: LLM replans if navigation fails
4. **Real manipulation**: Integrate MoveIt 2 for actual grasping
5. **Multi-room navigation**: Scale to 5+ rooms

## Common Issues

**"Whisper not detecting voice"**:
- Check microphone: `arecord -l`
- Test audio: `ros2 topic echo /voice/transcript` while speaking
- Reduce chunk duration to 2 seconds

**"LLM generates invalid JSON"**:
- Add validation before execution
- Use `format='json'` with Ollama
- Switch to GPT-4 for testing

**"Robot doesn't move"**:
- Verify Nav2 active: `ros2 service list | grep nav`
- Check semantic map coordinates match simulation
- Test manual navigation: `ros2 topic pub /goal_pose ...`

**Next Module**: [Capstone Integration](/docs/capstone/index) - Combine all modules into autonomous system.
