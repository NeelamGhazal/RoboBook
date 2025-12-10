---
title: "[Chapter Title - 3 to 60 characters]"
sidebar_position: [1-21, unique number]
---

# [Chapter Title]

## Learning Objectives

- [Action verb] [specific skill/concept related to chapter topic]
- [Action verb] [specific skill/concept related to chapter topic]
- [Action verb] [specific skill/concept related to chapter topic]
- [Optional: 4th objective]
- [Optional: 5th objective]

**Valid action verbs**: implement, explain, configure, analyze, design, build, test, debug, deploy, integrate, evaluate, compare, demonstrate, create, modify, troubleshoot

## Prerequisites

### Knowledge Prerequisites
- [Link to prior chapter: `/docs/module-X-topic/chapter-Y-title`]
- [Basic concept: e.g., "Basic Python (variables, functions, classes)"]

### Software Prerequisites
- **Operating System**: Ubuntu 22.04 LTS (physical, VM, or WSL2)
- **ROS 2**: Humble Hawksbill ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- **Python**: 3.10+ (ships with Ubuntu 22.04)
- **Packages**: `sudo apt install [package-name]`

### Hardware Prerequisites
[Only include if chapter requires specific hardware]
- **For Module 3 (Isaac Sim)**: NVIDIA GPU with CUDA 11.8+ support
- **For Module 2 (Gazebo)**: Discrete GPU recommended

### Installation Verification
Run this command to verify setup:
```bash
[command to test installation]
```

Expected output: `[exact output text]`

## Introduction

[2-3 paragraphs setting context]

[Paragraph 1: Start with an analogy or real-world example connecting the technical concept to familiar systems]

[Paragraph 2: Explain why this topic matters in Physical AI and robotics applications]

[Paragraph 3 (optional): Preview what learners will build/understand by the end]

## Theory

### [Concept 1]

[Explanation paragraph 1: Define the concept clearly]

[Explanation paragraph 2: Include a practical analogy comparing to physical/familiar systems]

[Insert Mermaid diagram here - see templates/mermaid-*-template.md]

### [Concept 2]

[Repeat structure for additional concepts]

## Code Examples

### Example 1: [Brief Descriptive Title]

[1-2 sentences explaining what this code demonstrates]

```python
# Complete, runnable Python code
# Must include all imports, initialization, and execution
import rclpy
from rclpy.node import Node


class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        # Implementation


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[Exact terminal output or description of visual result]
```

### Example 2: [Brief Descriptive Title] (if applicable)

[Additional code example following same structure]

## Exercises

1. **[Exercise Title]**: [Clear instructions]. Expected output: [description] or Success criteria: [measurable outcome]

2. **[Exercise Title]**: [Clear instructions]. Expected output: [description] or Success criteria: [measurable outcome]

3. **[Exercise Title - Optional]**: [Clear instructions]. Expected output: [description] or Success criteria: [measurable outcome]

## Summary

[2-3 sentences recapping the key learnings from this chapter]

## Next Steps

[Link to next chapter: `/docs/module-X-topic/chapter-Y-title`] - [Brief preview of next topic]
