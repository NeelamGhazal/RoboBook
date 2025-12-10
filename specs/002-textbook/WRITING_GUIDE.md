# Writing Guide: Textbook Chapter Creation

**Feature**: 002-textbook | **Purpose**: Comprehensive guide for creating textbook chapters

---

## Quick Reference

- **Template Location**: `specs/002-textbook/templates/chapter-template.md`
- **Code Template**: `specs/002-textbook/templates/code-example-template.py`
- **Mermaid Templates**: `specs/002-textbook/templates/mermaid-*-template.md`
- **Validation**: Run `npm run build` in `website/` to check for errors

---

## Chapter Creation Process

### Step 1: Copy Template

```bash
# Copy chapter template to correct module directory
cp specs/002-textbook/templates/chapter-template.md website/docs/module-X-topic/chapter-Y-title.md
```

### Step 2: Fill Frontmatter

```yaml
---
title: "ROS 2 Topics and Publishers"  # Descriptive, 3-60 chars
sidebar_position: 3                    # Unique number 1-21
---
```

**Sidebar Position Guide**:
- 1: Introduction
- 2-6: Module 1 (ROS 2)
- 7-11: Module 2 (Gazebo & Unity)
- 12-16: Module 3 (NVIDIA Isaac)
- 17-21: Module 4 (VLA)

### Step 3: Write Learning Objectives

✅ **Good Examples**:
- "Implement a ROS 2 publisher node using rclpy"
- "Explain the difference between topics and services"
- "Configure Gazebo physics parameters for realistic simulation"

❌ **Bad Examples**:
- "Learn about ROS 2 topics" (not measurable)
- "Understand publishers" (too vague)
- "Know how to use rclpy" (not specific)

**Action Verb List**: implement, explain, configure, analyze, design, build, test, debug, deploy, integrate, evaluate, compare, demonstrate, create, modify, troubleshoot

### Step 4: Document Prerequisites

**Structure**:
1. **Knowledge**: Link to prior chapters with absolute paths
2. **Software**: List OS, ROS version, Python version with exact versions
3. **Hardware**: Only if chapter requires specific hardware
4. **Verification**: Include command to test setup

**Example**:
```markdown
### Knowledge Prerequisites
- [Chapter 2: ROS 2 Topics](/docs/module-1-ros2/chapter-2-topics-pubsub)
- Basic Python (variables, functions, classes)

### Software Prerequisites
- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Packages**: `sudo apt install ros-humble-rclpy`

### Installation Verification
\`\`\`bash
python3 -c "import rclpy; print('ROS 2 ready!')"
\`\`\`
Expected output: `ROS 2 ready!`
```

### Step 5: Write Introduction with Analogy

**Structure** (2-3 paragraphs):
1. Start with relatable analogy
2. Explain why topic matters in robotics
3. Preview what learner will build

**Example**:
> Imagine a radio station broadcasting music. The station (publisher) sends signals over a frequency (topic), and anyone with a radio (subscriber) can listen. They don't know each other—the radio waves carry the message.
>
> ROS 2 topics work the same way. Publishers broadcast messages, and subscribers listen. This decoupling allows robot components to communicate without tight dependencies, making systems modular.
>
> In this chapter, you'll create a publisher that broadcasts messages, just like that radio station.

### Step 6: Write Theory Section

**For each concept**:
1. Define the concept clearly (1-2 paragraphs)
2. Include a practical analogy
3. Add a Mermaid diagram

**Mermaid Diagram Guidelines**:
- Use appropriate type (flowchart, sequence, state, class)
- Include `%%{title: "Description"}%%` for accessibility
- Keep to max 10-12 nodes
- Test rendering with `npm run build`

**Example**:
```markdown
### The Publish-Subscribe Pattern

ROS 2 uses pub/sub for asynchronous communication. Unlike function calls where a caller waits, publishers send messages without knowing who's listening.

Think of a newspaper: the publisher prints the paper (message) and distributes it. Subscribers buy the paper. The publisher doesn't know who reads it, and subscribers don't need to contact the publisher directly.

\`\`\`mermaid
%%{title: "ROS 2 Pub/Sub Architecture"}%%
graph LR
    A[Publisher] -->|topic: /data| B[Middleware]
    B --> C[Subscriber 1]
    B --> D[Subscriber 2]
\`\`\`
```

### Step 7: Write Code Examples

**Requirements**:
- Complete, runnable code (includes imports, main(), etc.)
- PEP 8 compliant (run `black` and `flake8`)
- 5-100 lines
- Includes comments explaining key parts
- Followed by **Expected Output** section

**Example**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
^C
```

### Step 8: Write Exercises

**Structure** (2-3 exercises):
1. **Title**: Bold, descriptive
2. **Instructions**: Clear, actionable steps
3. **Success Criteria**: "Expected output: ..." or "Success criteria: ..."

**Progression**:
- Early chapters: Guided (step-by-step)
- Mid chapters: Semi-guided (hints provided)
- Late chapters: Open-ended (problem statement only)

**Example**:
```markdown
1. **Modify Publisher Rate**: Change the MinimalPublisher to send messages every 0.5 seconds instead of 1.0 seconds. Expected output: Messages appear twice as fast (2 per second).

2. **Add Counter Reset**: Modify the publisher to reset counter to 0 every 10 messages. Success criteria: Console shows "Hello World: 0" after every 10 messages.
```

### Step 9: Write Summary & Next Steps

**Summary** (2-3 sentences):
- Recap key learnings
- Reinforce main concepts

**Next Steps**:
- Link to next chapter
- Brief preview of next topic

**Example**:
```markdown
## Summary

ROS 2 topics enable asynchronous communication via publish-subscribe. Publishers broadcast messages without knowing subscribers. This architecture allows modular robot systems with decoupled components.

## Next Steps

[Chapter 4: ROS 2 Subscribers](/docs/module-1-ros2/chapter-4-subscribers) - Learn how to receive and process published messages.
```

---

## Validation Checklist

Before submitting your chapter, verify:

### Content
- [ ] Word count 2000-3000 (excluding code, frontmatter, output)
- [ ] Introduction has at least one analogy
- [ ] Theory explains "why" not just "what"
- [ ] All technical terms defined on first use

### Structure
- [ ] Frontmatter: title (3-60 chars), sidebar_position (1-21, unique)
- [ ] 8 sections in order: Objectives, Prerequisites, Introduction, Theory, Code, Exercises, Summary, Next Steps
- [ ] Learning objectives: 3-5 items, action verbs
- [ ] Prerequisites: Links, versions stated
- [ ] Code: 1+, \`\`\`python language tag
- [ ] Expected output after each code block
- [ ] Mermaid: 1+, has title `%%{title: "..."}%%`
- [ ] Exercises: 2-3, have success criteria

### Code Quality
- [ ] Runs on Ubuntu 22.04 + ROS 2 Humble
- [ ] Passes `black --check`
- [ ] Passes `flake8`
- [ ] Output matches expected
- [ ] Completes within 30 seconds

### Technical
- [ ] No broken links (all `/docs/...` exist)
- [ ] Mermaid renders (`npm run build`)
- [ ] Prerequisites match dependencies
- [ ] Exercises solvable with chapter content

---

## Common Pitfalls

### 1. Non-Measurable Objectives
❌ "Learn about topics"
✅ "Implement a publisher using rclpy"

### 2. Incomplete Code
❌ Snippet without imports
✅ Complete script with all dependencies

### 3. Missing Output
❌ Code block alone
✅ Code + "**Expected Output**:" section

### 4. Vague Prerequisites
❌ "ROS 2 knowledge"
✅ "[Chapter 2](/docs/...)"

### 5. Complex Diagrams
❌ 20-node flowchart
✅ 8-node diagram, split if needed

### 6. No Success Criteria in Exercises
❌ "Modify the publisher"
✅ "Modify to send 10 messages. Expected: Terminal shows 'Hello World: 9' as final message"

---

## Tools and Commands

### Format Code
```bash
# Format Python code
black code.py

# Check formatting without modifying
black --check code.py
```

### Lint Code
```bash
# Lint Python code
flake8 code.py
```

### Test Code Execution
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run code
python3 code.py
```

### Validate Chapter in Docker
```bash
# Build validator image (one-time)
cd .docker/textbook-validator
docker build -t textbook-validator .

# Validate specific chapter
docker run --rm -v $(pwd):/workspace textbook-validator \
  /usr/local/bin/validate-chapter.sh /workspace/website/docs/module-1-ros2/chapter-1.md
```

### Build Docusaurus Site
```bash
cd website
npm run build  # Check for broken links, Mermaid errors
```

---

## Writing Style Guide

### Tone
- **Conversational but professional**: "You'll create..." not "One creates..."
- **Active voice**: "The publisher sends" not "Messages are sent by"
- **Direct**: Avoid "it is important to note that"

### Analogies
- **Domain-appropriate**: Use physical/familiar systems
- **Consistent**: Don't mix metaphors (radio → newspaper → phone in same chapter)
- **Clear mapping**: Explain what each part represents

### Technical Terms
- **Define on first use**: "LiDAR (Light Detection and Ranging) is..."
- **Link to references**: Use Docusaurus links for ROS 2 official docs
- **Avoid jargon**: If you must use jargon, explain it

### Code Comments
- **Why, not what**: Explain reasoning, not obvious operations
- **Key decisions**: Comment on non-trivial choices
- **Minimal**: Code should be self-explanatory

---

## Example Chapter: ROS 2 Minimal Publisher

See `specs/002-textbook/quickstart.md` for a complete example chapter demonstrating all guidelines.

---

## Getting Help

- **Template issues**: Check `specs/002-textbook/templates/`
- **Technical decisions**: See `specs/002-textbook/research.md`
- **Data model**: See `specs/002-textbook/data-model.md`
- **Validation contract**: See `specs/002-textbook/contracts/chapter-schema.md`
- **CI/CD failures**: Check `.github/workflows/validate-chapters.yml`

---

**Last Updated**: 2025-12-10
**Feature**: 002-textbook
