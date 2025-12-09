---
id: style-guide
title: Content Style Guide
sidebar_label: Style Guide
---

# Physical AI & Humanoid Robotics - Content Style Guide

This guide ensures consistent, high-quality content across all chapters following the project constitution.

## Constitutional Principles

All content MUST adhere to the [Project Constitution](../../.specify/memory/constitution.md):

1. **Practical Over Theoretical**: Every concept must have working, tested examples
2. **Learn By Doing**: Tutorial-driven content takes precedence
3. **Clarity First**: Flesch-Kincaid grade 10-12 readability
4. **Reproducibility**: All code/commands must work from clean environment
5. **Progressive Complexity**: Start simple, build to advanced (max 2-3 new concepts per chapter)
6. **Tool-Agnostic Mindset**: Explain "why" before "how"
7. **Consistency and Standards**: Unified terminology and formatting

## Readability Standards

### Flesch-Kincaid Grade Level: 10-12

**Target**: Technical content accessible to high school seniors and college freshmen

**Guidelines**:
- Average sentence length: 15-20 words
- Use active voice ("The robot moves" not "The robot is moved")
- Define jargon on first use
- Break complex ideas into shorter paragraphs (3-5 sentences max)

**Tools**:
```bash
# Check readability score
npx readability-cli docs/module-01-ros2/ros2-architecture.md
# Target: FK Grade 10-12
```

### Paragraph Structure

- **Max length**: 5 sentences or 100 words
- **First sentence**: Topic sentence stating main idea
- **Supporting sentences**: Examples, explanations, evidence
- **Transition**: Connect to next paragraph

### Sentence Structure

✅ **Good**: "ROS 2 uses a publish-subscribe pattern for communication between nodes."

❌ **Avoid**: "In ROS 2, the communication paradigm employed for inter-node data exchange leverages the publisher-subscriber architectural pattern."

## Terminology

### Standardized Terms

Use these exact terms (case-sensitive):

| Correct | Avoid |
|---------|-------|
| ROS 2 Humble | ROS2, ros2, ROS 2 humble |
| URDF (Unified Robot Description Format) | urdf, Urdf |
| Gazebo Classic | Gazebo 11, gazebo |
| NVIDIA Isaac Sim | Isaac Sim, isaac-sim |
| Physical AI | physical AI, PhysicalAI |
| Vision-Language-Action (VLA) | VLA model, vla |
| LLM (Large Language Model) | llm, AI model |

### First Use Definition Pattern

```markdown
Visual SLAM (Simultaneous Localization and Mapping) enables robots to build maps...
```

## Code Formatting

### Python (ROS 2)

**Style**: Black formatter with 100-character line length

```python
#!/usr/bin/env python3
"""
Module: hello_world
Description: Minimal ROS 2 node demonstrating basic functionality
"""

import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    """Simple example node for ROS 2 Humble."""

    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Requirements**:
- Docstrings for all classes and non-trivial functions
- Type hints for function parameters
- No commented-out code in final version
- Inline comments only for non-obvious logic

### Bash Commands

```bash
# Good: Clear, one command per line with comments
ros2 run hello_world hello_world_node

# Also good: Chained commands with && for dependencies
cd ~/ros2_ws && colcon build && source install/setup.bash

# Avoid: Multiple unrelated commands without explanation
ros2 topic list
ros2 node list
```

### URDF/XML

```xml
<!-- Humanoid robot base URDF -->
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Content Structure

### Chapter Template

Use `.templates/chapter-template.mdx` for all chapters. Required sections:

1. **Frontmatter** (YAML metadata)
2. **Learning Outcomes** (3-5 bullet points)
3. **Introduction** (2-3 paragraphs)
4. **Core Content** (2-4 major sections)
5. **Hands-On Tutorial** (step-by-step)
6. **Verification** (how to test)
7. **Summary** (key takeaways)
8. **Exercises** (beginner + intermediate)

### Section Lengths

- **Chapter**: 3,000-5,000 words
- **Section**: 500-1,500 words
- **Subsection**: 200-500 words

## Code Examples

### Complete, Runnable Code

✅ **Good**: Full example with all imports

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from ROS 2!'
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

❌ **Avoid**: Incomplete snippets with ellipsis

```python
class PublisherNode(Node):
    def __init__(self):
        # ... setup code here ...
```

### Expected Output

Always show expected output for commands:

```bash
$ ros2 run hello_world hello_world_node
[INFO] [hello_world_node]: Hello, ROS 2!
```

## Admonitions

Use sparingly (max 2-3 per chapter):

### Common Pitfalls

```markdown
:::caution Common Pitfall
Forgetting to source `install/setup.bash` after building is the #1 cause of "package not found" errors.

**Solution**: Add to ~/.bashrc: `source ~/ros2_ws/install/setup.bash`
:::
```

### Tips

```markdown
:::tip Pro Tip
Use `colcon build --symlink-install` to avoid rebuilding after Python changes.
:::
```

### Hardware Requirements

```markdown
:::info Hardware Note
This tutorial requires an NVIDIA RTX GPU. Cloud alternative: AWS g5.2xlarge (~$205/quarter).
:::
```

## Images & Diagrams

### File Naming

```
static/img/module-01/ros2-architecture-diagram.svg
static/img/getting-started/ubuntu-install-screenshot.png
```

### Alt Text (Accessibility)

```markdown
![ROS 2 architecture showing nodes, topics, and services](../img/module-01/ros2-architecture.svg)
```

### Optimization

- **PNG**: `<500KB` (use TinyPNG or ImageOptim)
- **SVG**: Preferred for diagrams
- **JPEG**: Avoid (use PNG for screenshots)

## Links

### Internal Links

```markdown
Continue to [ROS 2 Architecture](/docs/module-01-ros2/ros2-architecture)
```

### External Links

```markdown
Read more: [ROS 2 Documentation](https://docs.ros.org/en/humble/)
```

### Code Repository Links

```markdown
[View Code](https://github.com/umeradnan7106/physical-ai-code-examples/tree/main/module-01-ros2/hello_world)
```

## Quality Gates

Before submitting content, verify:

- [ ] **Readability**: FK grade 10-12 (check with readability-cli)
- [ ] **Code**: All examples tested on Ubuntu 22.04 + ROS 2 Humble
- [ ] **Linting**: Python code scores ≥8.0/10 with Pylint
- [ ] **Links**: All internal/external links functional
- [ ] **Images**: All images `<500KB`, have alt text
- [ ] **Build**: `npm run build` completes with zero warnings
- [ ] **Constitution**: Follows all 7 constitutional principles

## Review Checklist

Peer reviewers should check:

- [ ] Adheres to constitutional principles
- [ ] Readability FK 10-12
- [ ] No placeholder text ([TODO], [FILL IN])
- [ ] Code examples complete and tested
- [ ] Consistent terminology per style guide
- [ ] All acceptance scenarios from spec covered
- [ ] Troubleshooting section for common issues

## Questions?

- Book content issues: [GitHub Issues](https://github.com/umeradnan7106/physical-ai-robotics-book/issues)
- Code example issues: [Code Repo Issues](https://github.com/umeradnan7106/physical-ai-code-examples/issues)
