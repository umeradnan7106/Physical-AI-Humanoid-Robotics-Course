---
id: llm-planning
title: LLM Planning
sidebar_position: 3
description: Use GPT-4 and Claude APIs for robot task planning with prompt engineering.
keywords: [llm planning, gpt-4, claude, prompt engineering, structured outputs]
---

# LLM Planning with GPT-4 & Claude

**Natural language → Structured robot actions.**

## Why Use LLMs for Robot Planning?

**Traditional approach**: Hand-code every task ("if user says 'kitchen', navigate to (2.5, 3.0)")

**LLM approach**: LLM interprets intent and generates action plan from natural language

**Advantages**:
- **Flexibility**: Handle variations ("go to kitchen" = "head to the cooking area")
- **Context awareness**: Consider robot state, environment
- **Chain tasks**: "Get cup from kitchen, bring to table" → multi-step plan

## API Setup

### OpenAI GPT-4

**Install SDK**:
```bash
pip install openai
```

**Get API key**: https://platform.openai.com/api-keys

**Set environment variable**:
```bash
export OPENAI_API_KEY="sk-..."
```

### Anthropic Claude

**Install SDK**:
```bash
pip install anthropic
```

**Get API key**: https://console.anthropic.com/settings/keys

**Set environment variable**:
```bash
export ANTHROPIC_API_KEY="sk-ant-..."
```

## Prompt Engineering for Robotics

### System Prompt Template

```python
ROBOT_SYSTEM_PROMPT = """You are a robot task planner. Given a user command, generate a JSON action plan.

Available actions:
- navigate: Move to location (params: location_name)
- pick: Grasp object (params: object_id)
- place: Release object (params: location_name)
- wait: Pause execution (params: duration_seconds)

Available locations: kitchen, living_room, bedroom, table, shelf

Output format (JSON):
{
  "steps": [
    {"action": "navigate", "params": {"location_name": "kitchen"}},
    {"action": "pick", "params": {"object_id": "cup"}},
    {"action": "navigate", "params": {"location_name": "table"}},
    {"action": "place", "params": {"location_name": "table"}}
  ]
}

If the command is unclear, return:
{"error": "Clarification needed: <question>"}
"""
```

### GPT-4 Implementation

```python
from openai import OpenAI
import json

class GPT4Planner:
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)

    def plan_action(self, user_command: str, robot_state: dict = None) -> dict:
        messages = [
            {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
            {"role": "user", "content": user_command}
        ]

        # Add robot state context if available
        if robot_state:
            messages.insert(1, {
                "role": "system",
                "content": f"Current robot state: {json.dumps(robot_state)}"
            })

        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.0,  # Deterministic for safety
            response_format={"type": "json_object"}  # Enforce JSON output
        )

        return json.loads(response.choices[0].message.content)

# Usage
planner = GPT4Planner(api_key=os.getenv("OPENAI_API_KEY"))
plan = planner.plan_action("Bring me a cup from the kitchen")
print(plan)
# Output: {"steps": [{"action": "navigate", "params": {"location_name": "kitchen"}}, ...]}
```

### Claude Implementation

```python
from anthropic import Anthropic
import json

class ClaudePlanner:
    def __init__(self, api_key):
        self.client = Anthropic(api_key=api_key)

    def plan_action(self, user_command: str, robot_state: dict = None) -> dict:
        system_prompt = ROBOT_SYSTEM_PROMPT
        if robot_state:
            system_prompt += f"\n\nCurrent robot state: {json.dumps(robot_state)}"

        response = self.client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=1024,
            temperature=0.0,
            system=system_prompt,
            messages=[
                {"role": "user", "content": user_command}
            ]
        )

        return json.loads(response.content[0].text)

# Usage
planner = ClaudePlanner(api_key=os.getenv("ANTHROPIC_API_KEY"))
plan = planner.plan_action("Go to the living room and wait 5 seconds")
```

## Advanced: Few-Shot Prompting

**Problem**: LLM generates invalid actions

**Solution**: Provide examples in prompt

```python
FEW_SHOT_EXAMPLES = """
Example 1:
User: "Get the red box from the shelf"
Output: {
  "steps": [
    {"action": "navigate", "params": {"location_name": "shelf"}},
    {"action": "pick", "params": {"object_id": "red_box"}}
  ]
}

Example 2:
User: "Put the cup on the table"
Output: {
  "steps": [
    {"action": "pick", "params": {"object_id": "cup"}},
    {"action": "navigate", "params": {"location_name": "table"}},
    {"action": "place", "params": {"location_name": "table"}}
  ]
}

Example 3:
User: "Go to Mars"
Output: {"error": "Clarification needed: Mars is not a known location. Available: kitchen, living_room, bedroom, table, shelf"}
"""

# Add to system prompt
ROBOT_SYSTEM_PROMPT_WITH_EXAMPLES = ROBOT_SYSTEM_PROMPT + "\n\n" + FEW_SHOT_EXAMPLES
```

## ROS 2 Integration

**planner_node.py**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Subscribe to voice transcripts
        self.transcript_sub = self.create_subscription(
            String, '/voice/transcript', self.transcript_callback, 10
        )

        # Publish action plans
        self.plan_pub = self.create_publisher(String, '/robot/action_plan', 10)

        # Initialize LLM planner (swap GPT4Planner/ClaudePlanner)
        self.planner = GPT4Planner(api_key=os.getenv("OPENAI_API_KEY"))

        self.get_logger().info('LLM Planner ready')

    def transcript_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f'Planning for: "{user_command}"')

        try:
            plan = self.planner.plan_action(user_command)

            if "error" in plan:
                self.get_logger().warn(f'Planning error: {plan["error"]}')
            else:
                # Publish plan as JSON string
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)
                self.get_logger().info(f'Published plan: {plan}')

        except Exception as e:
            self.get_logger().error(f'LLM error: {str(e)}')

def main():
    rclpy.init()
    node = LLMPlannerNode()
    rclpy.spin(node)
```

**Run**:
```bash
ros2 run llm_planner planner_node
```

## Structured Outputs (OpenAI Function Calling)

**Better approach**: Use function calling for guaranteed JSON schema

```python
from openai import OpenAI

client = OpenAI()

tools = [
    {
        "type": "function",
        "function": {
            "name": "execute_robot_plan",
            "description": "Generate a multi-step robot action plan",
            "parameters": {
                "type": "object",
                "properties": {
                    "steps": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "action": {"type": "string", "enum": ["navigate", "pick", "place", "wait"]},
                                "params": {"type": "object"}
                            },
                            "required": ["action", "params"]
                        }
                    }
                },
                "required": ["steps"]
            }
        }
    }
]

response = client.chat.completions.create(
    model="gpt-4o",
    messages=[
        {"role": "system", "content": ROBOT_SYSTEM_PROMPT},
        {"role": "user", "content": "Bring me a cup from the kitchen"}
    ],
    tools=tools,
    tool_choice={"type": "function", "function": {"name": "execute_robot_plan"}}
)

# Extract function call arguments (guaranteed valid JSON)
plan = json.loads(response.choices[0].message.tool_calls[0].function.arguments)
```

## Cost Optimization

**API costs**:
- GPT-4o: $2.50 per 1M input tokens, $10 per 1M output tokens
- Claude Sonnet: $3 per 1M input tokens, $15 per 1M output tokens

**Tips**:
1. **Cache system prompts** (Anthropic supports prompt caching)
2. **Use smaller models for simple tasks** (GPT-4o-mini: 15× cheaper)
3. **Limit max_tokens** to 256 for simple plans
4. **Batch requests** when latency isn't critical

**Example with caching** (Anthropic):
```python
response = client.messages.create(
    model="claude-sonnet-4-20250514",
    max_tokens=256,
    system=[
        {
            "type": "text",
            "text": ROBOT_SYSTEM_PROMPT,
            "cache_control": {"type": "ephemeral"}  # Cache this prompt
        }
    ],
    messages=[{"role": "user", "content": user_command}]
)
```

## Error Handling

**Validate LLM output before execution**:
```python
def validate_plan(plan: dict) -> bool:
    if "error" in plan:
        return False

    if "steps" not in plan or not isinstance(plan["steps"], list):
        return False

    valid_actions = {"navigate", "pick", "place", "wait"}
    for step in plan["steps"]:
        if step["action"] not in valid_actions:
            return False
        if "params" not in step:
            return False

    return True

# In planner_node.py
plan = self.planner.plan_action(user_command)
if validate_plan(plan):
    self.plan_pub.publish(String(data=json.dumps(plan)))
else:
    self.get_logger().error(f'Invalid plan: {plan}')
```

**Next**: [Local LLMs with Ollama](/docs/module-04-vla/ollama-local-llms)
