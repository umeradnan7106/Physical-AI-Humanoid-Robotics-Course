---
id: action-execution
title: Action Execution
sidebar_position: 5
description: Parse LLM outputs and execute robot actions via ROS 2 action servers.
keywords: [action execution, ros 2 actions, motion primitives, navigation]
---

# Action Execution

**LLM plans → Robot movements.**

## Action Executor Architecture

**Flow**:
1. LLM generates JSON plan: `{"steps": [{"action": "navigate", "params": {...}}]}`
2. Executor parses JSON and validates
3. Executor calls appropriate ROS 2 action server
4. Wait for completion, then execute next step

## Define Motion Primitives

**Motion primitives** = atomic robot actions

```python
from enum import Enum

class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    WAIT = "wait"

class ActionParams:
    """Base class for action parameters"""
    pass

class NavigateParams(ActionParams):
    location_name: str  # "kitchen", "table", etc.

class PickParams(ActionParams):
    object_id: str  # "cup", "red_box", etc.

class PlaceParams(ActionParams):
    location_name: str

class WaitParams(ActionParams):
    duration_seconds: float
```

## Action Executor Node

**executor_node.py**:
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Subscribe to action plans from LLM planner
        self.plan_sub = self.create_subscription(
            String, '/robot/action_plan', self.plan_callback, 10
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Semantic map (location_name → coordinates)
        self.semantic_map = {
            'kitchen': (2.5, 3.0),
            'living_room': (-1.0, 2.0),
            'bedroom': (5.0, -2.0),
            'table': (1.0, 1.0),
            'shelf': (3.0, -1.0)
        }

        self.get_logger().info('Action Executor ready')

    def plan_callback(self, msg):
        try:
            plan = json.loads(msg.data)

            if "steps" not in plan:
                self.get_logger().error(f'Invalid plan: missing "steps"')
                return

            self.get_logger().info(f'Executing plan with {len(plan["steps"])} steps')

            for i, step in enumerate(plan["steps"]):
                self.get_logger().info(f'Step {i+1}: {step}')
                success = self.execute_step(step)

                if not success:
                    self.get_logger().error(f'Step {i+1} failed, aborting plan')
                    return

            self.get_logger().info('Plan completed successfully!')

        except Exception as e:
            self.get_logger().error(f'Execution error: {str(e)}')

    def execute_step(self, step: dict) -> bool:
        action = step.get("action")
        params = step.get("params", {})

        if action == "navigate":
            return self.execute_navigate(params)
        elif action == "pick":
            return self.execute_pick(params)
        elif action == "place":
            return self.execute_place(params)
        elif action == "wait":
            return self.execute_wait(params)
        else:
            self.get_logger().error(f'Unknown action: {action}')
            return False

    def execute_navigate(self, params: dict) -> bool:
        location_name = params.get("location_name")

        if location_name not in self.semantic_map:
            self.get_logger().error(f'Unknown location: {location_name}')
            return False

        x, y = self.semantic_map[location_name]
        self.get_logger().info(f'Navigating to {location_name} at ({x}, {y})')

        # Create Nav2 goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        # Send goal and wait
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(f'Arrived at {location_name}')
        return True

    def execute_pick(self, params: dict) -> bool:
        object_id = params.get("object_id")
        self.get_logger().info(f'Picking up {object_id}')

        # TODO: Call manipulation action server
        # For now, simulate pick
        time.sleep(2.0)
        self.get_logger().info(f'Picked up {object_id}')
        return True

    def execute_place(self, params: dict) -> bool:
        location_name = params.get("location_name")
        self.get_logger().info(f'Placing object at {location_name}')

        # TODO: Call manipulation action server
        time.sleep(2.0)
        self.get_logger().info(f'Placed object at {location_name}')
        return True

    def execute_wait(self, params: dict) -> bool:
        duration = params.get("duration_seconds", 1.0)
        self.get_logger().info(f'Waiting {duration} seconds')
        time.sleep(duration)
        return True

def main():
    rclpy.init()
    node = ActionExecutorNode()
    rclpy.spin(node)
```

**Run**:
```bash
ros2 run action_executor executor_node
```

## Test End-to-End Pipeline

**Terminal 1**: Start Whisper
```bash
ros2 run whisper_ros whisper_node
```

**Terminal 2**: Start LLM Planner
```bash
export LLM_PROVIDER=ollama  # or gpt4
ros2 run llm_planner planner_node
```

**Terminal 3**: Start Action Executor
```bash
ros2 run action_executor executor_node
```

**Terminal 4**: Launch Isaac Sim/Gazebo with Nav2

**Test**: Speak "Go to the kitchen and wait 5 seconds"

**Expected flow**:
1. Whisper transcribes → `/voice/transcript`: "Go to the kitchen and wait 5 seconds"
2. LLM Planner generates → `/robot/action_plan`:
   ```json
   {
     "steps": [
       {"action": "navigate", "params": {"location_name": "kitchen"}},
       {"action": "wait", "params": {"duration_seconds": 5.0}}
     ]
   }
   ```
3. Action Executor calls Nav2 → robot navigates to kitchen
4. Action Executor waits 5 seconds

## Advanced: Manipulation Actions

**MoveIt 2 integration** (for pick/place):

```python
from moveit_msgs.action import MoveGroup

class ActionExecutorNode(Node):
    def __init__(self):
        # ... (previous code)

        # MoveIt action client
        self.moveit_client = ActionClient(self, MoveGroup, 'move_action')

    def execute_pick(self, params: dict) -> bool:
        object_id = params.get("object_id")

        # 1. Open gripper
        self.set_gripper(open=True)

        # 2. Move to pre-grasp pose
        object_pose = self.get_object_pose(object_id)  # From object detection
        pre_grasp_pose = self.compute_pre_grasp(object_pose)
        self.move_to_pose(pre_grasp_pose)

        # 3. Approach object
        grasp_pose = object_pose
        self.move_to_pose(grasp_pose)

        # 4. Close gripper
        self.set_gripper(open=False)

        # 5. Lift object
        lift_pose = self.compute_lift_pose(grasp_pose)
        self.move_to_pose(lift_pose)

        return True
```

## Error Handling & Recovery

**Retry on failure**:
```python
def execute_step(self, step: dict, max_retries=3) -> bool:
    for attempt in range(max_retries):
        success = self._execute_step_once(step)

        if success:
            return True

        self.get_logger().warn(f'Step failed, retry {attempt+1}/{max_retries}')
        time.sleep(2.0)

    return False
```

**Timeout protection**:
```python
def execute_navigate(self, params: dict, timeout=60.0) -> bool:
    # ... send goal ...

    # Wait with timeout
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

    if not result_future.done():
        self.get_logger().error('Navigation timeout')
        goal_handle.cancel_goal_async()
        return False

    return True
```

## Validation Before Execution

**Safety checks**:
```python
def validate_plan(plan: dict) -> tuple[bool, str]:
    if "steps" not in plan:
        return False, "Missing 'steps' field"

    if not isinstance(plan["steps"], list):
        return False, "'steps' must be a list"

    if len(plan["steps"]) > 10:
        return False, "Plan too long (max 10 steps)"

    valid_actions = {"navigate", "pick", "place", "wait"}
    for step in plan["steps"]:
        if "action" not in step:
            return False, "Step missing 'action'"

        if step["action"] not in valid_actions:
            return False, f"Invalid action: {step['action']}"

        if "params" not in step:
            return False, "Step missing 'params'"

    return True, "Valid"

# In plan_callback
valid, error_msg = validate_plan(plan)
if not valid:
    self.get_logger().error(f'Invalid plan: {error_msg}')
    return
```

**Next**: [VLA Capstone Project](/docs/module-04-vla/project-vla-pipeline)
