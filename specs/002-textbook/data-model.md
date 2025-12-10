# Data Model: AI-Native Textbook

**Feature**: 002-textbook | **Date**: 2025-12-09
**Purpose**: Define the structure of educational content entities (chapters, modules, code examples)

---

## Entity: Chapter

**Description**: A single lesson in the textbook. Primary content unit containing learning objectives, theory, code examples, and exercises.

### Attributes

| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| `title` | string | Required, 3-60 chars | Chapter display name (e.g., "ROS 2 Nodes and Architecture") |
| `sidebar_position` | integer | Required, 1-21 | Position in sidebar (1=intro, 2-6=Module 1, 7-11=Module 2, etc.) |
| `module_id` | string | Optional, enum | Module identifier (`intro`, `module-1-ros2`, `module-2-simulation`, `module-3-isaac`, `module-4-vla`) |
| `learning_objectives` | array<string> | Required, 3-5 items | Measurable learning goals starting with action verbs (implement, explain, configure) |
| `prerequisites` | object | Required | Knowledge, software, and hardware requirements |
| `prerequisites.knowledge` | array<string> | Optional | Links to prior chapters or concepts |
| `prerequisites.software` | array<string> | Required | Software dependencies with versions (ROS 2 Humble, Python 3.10+) |
| `prerequisites.hardware` | array<string> | Optional | Hardware requirements if applicable (NVIDIA GPU, etc.) |
| `content_sections` | array<Section> | Required, ≥4 sections | Main body: Introduction, Theory (with subsections), Code Examples, Exercises |
| `word_count` | integer | 2000-3000 | Content word count excluding code blocks, frontmatter, output sections |
| `code_examples` | array<CodeExample> | Required, ≥1 | Executable Python code blocks |
| `mermaid_diagrams` | array<MermaidDiagram> | Required, ≥1 | Visual diagrams for concepts |
| `exercises` | array<Exercise> | Required, 2-3 items | Hands-on tasks reinforcing learning objectives |
| `next_chapter_link` | string | Optional | Relative path to next chapter (`/docs/module-1-ros2/chapter-2-topics`) |

### Relationships

- **Belongs to**: One Module (or `intro` standalone)
- **Prerequisites**: Links to zero or more prior Chapters
- **Contains**: 1+ CodeExamples, 1+ MermaidDiagrams, 2-3 Exercises
- **Validates against**: ChapterSchema (JSON Schema in contracts/)

### State Transitions

1. **Draft** → Content being written, may have placeholders
2. **Review** → All sections complete, awaiting technical review
3. **Validated** → Code examples tested in Docker, Mermaid syntax verified
4. **Published** → Merged to main branch, live on Docusaurus site

### Example Instance

```yaml
title: "ROS 2 Nodes and Architecture"
sidebar_position: 2
module_id: "module-1-ros2"
learning_objectives:
  - "Explain the purpose of ROS 2 nodes in distributed robotics systems"
  - "Implement a minimal ROS 2 node using rclpy"
  - "Configure node parameters for runtime behavior modification"
prerequisites:
  knowledge:
    - "Basic Python (variables, functions, classes)"
    - "/docs/intro - Understanding of Physical AI concepts"
  software:
    - "Ubuntu 22.04 LTS"
    - "ROS 2 Humble Hawksbill"
    - "Python 3.10+"
  hardware: []
content_sections:
  - type: "introduction"
    word_count: 250
  - type: "theory"
    subsections: ["What is a ROS 2 Node?", "Node Lifecycle", "Node Graph"]
    word_count: 1200
  - type: "code_examples"
    count: 3
  - type: "exercises"
    count: 3
word_count: 2450
code_examples: [...]  # See CodeExample entity below
mermaid_diagrams: [...]  # See MermaidDiagram entity below
exercises: [...]  # See Exercise entity below
next_chapter_link: "/docs/module-1-ros2/chapter-2-topics-pubsub"
```

---

## Entity: Module

**Description**: A major learning unit containing 5 chapters. Represents a thematic grouping (e.g., ROS 2, Gazebo/Unity, Isaac, VLA).

### Attributes

| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| `module_id` | string | Required, unique | Identifier (`module-1-ros2`, `module-2-simulation`, `module-3-isaac`, `module-4-vla`) |
| `title` | string | Required | Display name (e.g., "Module 1: The Robotic Nervous System") |
| `description` | string | Required, 1-2 sentences | Brief module overview |
| `chapter_count` | integer | Required, always 5 | Number of chapters in module |
| `chapters` | array<Chapter> | Required, length 5 | Ordered list of chapters |
| `sidebar_position_range` | object | Derived | Sidebar positions spanned (e.g., {start: 2, end: 6} for Module 1) |

### Relationships

- **Contains**: Exactly 5 Chapters (per Principle II)
- **Sequential dependency**: Module N prerequisites include Module N-1 completion (except Module 1)

### Example Instance

```yaml
module_id: "module-1-ros2"
title: "Module 1: The Robotic Nervous System (ROS 2)"
description: "Learn ROS 2 fundamentals including nodes, topics, services, rclpy, and robot description with URDF."
chapter_count: 5
chapters:
  - chapter-1-nodes-architecture.md
  - chapter-2-topics-pubsub.md
  - chapter-3-services-clients.md
  - chapter-4-rclpy-python-client.md
  - chapter-5-urdf-robot-description.md
sidebar_position_range: {start: 2, end: 6}
```

---

## Entity: CodeExample

**Description**: An executable Python code block within a chapter, demonstrating a concept or pattern.

### Attributes

| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| `title` | string | Required | Brief description (e.g., "Example 1: Minimal ROS 2 Publisher") |
| `language` | string | Required, always "python" | Programming language (fixed to Python for this textbook) |
| `code` | string | Required, 5-100 lines | Complete, runnable Python code with imports, initialization, execution |
| `expected_output` | string | Required | Exact terminal output or description of visual result |
| `execution_time_seconds` | integer | <30 | Max time for code to complete (performance goal) |
| `dependencies` | array<string> | Required | Python packages needed (e.g., ["rclpy", "std_msgs"]) |
| `setup_commands` | array<string> | Optional | Shell commands to run before code (e.g., `ros2 run ...`) |
| `pep8_compliant` | boolean | Required, always true | Whether code passes black + flake8 |

### Relationships

- **Belongs to**: One Chapter
- **Validates via**: Docker container (Ubuntu 22.04 + ROS 2 Humble)
- **May depend on**: Previous CodeExamples in same or prior chapters

### Example Instance

```yaml
title: "Example 1: Minimal ROS 2 Publisher"
language: "python"
code: |
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

expected_output: |
  [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
  ^C (Ctrl-C to stop)

execution_time_seconds: 5
dependencies: ["rclpy", "std_msgs"]
setup_commands: []
pep8_compliant: true
```

---

## Entity: MermaidDiagram

**Description**: A visual diagram explaining architecture, data flow, or state machine concepts.

### Attributes

| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| `type` | string | Required, enum | Diagram type (`flowchart`, `sequence`, `state`, `class`) |
| `title` | string | Required | Alt text for accessibility (e.g., "ROS 2 pub/sub architecture") |
| `mermaid_code` | string | Required | Valid Mermaid syntax |
| `node_count` | integer | ≤12 | Number of nodes/participants (keep simple per research.md) |
| `validates` | boolean | Required, always true | Whether Mermaid syntax is valid (checked during Docusaurus build) |

### Relationships

- **Belongs to**: One Chapter
- **Illustrates**: Specific concept from Theory section
- **Validates via**: Docusaurus build process (Mermaid plugin)

### Example Instance

```yaml
type: "flowchart"
title: "ROS 2 Publisher-Subscriber Communication"
mermaid_code: |
  graph LR
      A[Publisher Node] -->|topic: /sensor_data| B[ROS 2 Middleware DDS]
      B --> C[Subscriber Node 1]
      B --> D[Subscriber Node 2]
      C --> E[Process Data]
      D --> F[Log Data]
node_count: 6
validates: true
```

---

## Entity: Exercise

**Description**: A hands-on task at the end of a chapter to reinforce learning objectives.

### Attributes

| Attribute | Type | Constraints | Description |
|-----------|------|-------------|-------------|
| `title` | string | Required | Brief description (e.g., "Exercise 1: Modify Publisher Rate") |
| `type` | string | Required, enum | Task type (`guided`, `semi-guided`, `open-ended`, `discussion`) |
| `difficulty` | string | Required, enum | Difficulty level (`beginner`, `intermediate`, `advanced`) |
| `instructions` | string | Required, 2-5 sentences | Clear task description with success criteria |
| `hints` | array<string> | Optional | Clues for semi-guided exercises |
| `estimated_time_minutes` | integer | 10-60 | Expected completion time |
| `solution_available` | boolean | Required | Whether solution exists in solutions/ directory |

### Relationships

- **Belongs to**: One Chapter
- **Reinforces**: One or more learning objectives from same chapter

### Example Instance

```yaml
title: "Exercise 1: Modify Publisher Rate"
type: "guided"
difficulty: "beginner"
instructions: "Modify the MinimalPublisher to send messages every 0.5 seconds instead of 1.0 seconds. Run the node and verify the message rate doubled. Success criteria: Messages appear twice as fast in terminal."
hints:
  - "Look for the create_timer() call in __init__()"
  - "The first argument is the timer period in seconds"
estimated_time_minutes: 15
solution_available: true
```

---

## Validation Rules

### Chapter Validation

1. **Frontmatter**: Must have `title` and `sidebar_position`
2. **Sections**: Must include (in order): Learning Objectives, Prerequisites, Introduction, Theory, Code Examples, Exercises
3. **Word count**: 2000-3000 words excluding code blocks, frontmatter, expected output
4. **Learning objectives**: 3-5 items, each starting with measurable verb
5. **Code examples**: At least 1, all must be PEP 8 compliant and executable
6. **Mermaid diagrams**: At least 1, must render without Docusaurus build errors
7. **Exercises**: 2-3 items with clear success criteria

### Module Validation

1. **Chapter count**: Exactly 5 chapters per module
2. **Sidebar positions**: Sequential (e.g., Module 1 chapters use positions 2-6)
3. **Naming**: Directory matches `module-{N}-{theme}` pattern
4. **Completeness**: All 5 chapter files exist and validate individually

### CodeExample Validation

1. **Executability**: Runs without errors in Ubuntu 22.04 + ROS 2 Humble Docker container
2. **Output match**: Actual output matches expected output (fuzzy match for timestamps/PIDs)
3. **Timeout**: Completes within 30 seconds
4. **PEP 8**: Passes `black --check` and `flake8`
5. **Dependencies**: All imports listed in `dependencies` array

### Cross-Entity Validation

1. **Prerequisites**: All chapter links in `prerequisites.knowledge` point to existing chapters
2. **Sidebar positions**: No gaps (1, 2, 3, ... 21) and no duplicates
3. **Module ordering**: Module 2 prerequisites include Module 1, Module 3 includes 1+2, Module 4 includes 1+2+3
4. **Next chapter links**: Form valid chain from intro → Module 1 Ch 1 → ... → Module 4 Ch 5 → null

---

## Textbook Content Structure (Full)

```
21 Chapters Total:
├── 1 Introduction (sidebar_position: 1, module_id: "intro")
├── Module 1: ROS 2 (sidebar_positions: 2-6, module_id: "module-1-ros2")
│   ├── 5 Chapters
│   └── Each contains: 3-5 objectives, 1+ code examples, 1+ Mermaid diagrams, 2-3 exercises
├── Module 2: Gazebo/Unity (sidebar_positions: 7-11, module_id: "module-2-simulation")
│   ├── 5 Chapters
│   └── Same structure as Module 1
├── Module 3: Isaac (sidebar_positions: 12-16, module_id: "module-3-isaac")
│   ├── 5 Chapters
│   └── Same structure as Module 1
└── Module 4: VLA (sidebar_positions: 17-21, module_id: "module-4-vla")
    ├── 5 Chapters
    └── Same structure as Module 1

Total counts:
- 21 chapters
- 63-105 learning objectives (3-5 per chapter)
- 21+ code examples (1+ per chapter)
- 21+ Mermaid diagrams (1+ per chapter)
- 42-63 exercises (2-3 per chapter)
- ~50,000 words total (2000–3000 per chapter × 21)
```

---

## Usage Notes

- **Chapter template**: See `quickstart.md` for markdown template enforcing this structure
- **Schema validation**: See `contracts/chapter-schema.md` for JSON Schema used in automated checks
- **Docker validation**: See `.docker/textbook-validator/` for code execution testing
- **Sidebar config**: See `website/sidebars.ts` for Docusaurus navigation configuration

All entities align with:
- ✅ Principle I: Educational Quality First (learning objectives, exercises, word count)
- ✅ Principle II: Structured Content Architecture (21 chapters, 4 modules, template adherence)
- ✅ Principle III: Code Correctness & Testability (PEP 8, executable, Docker validation)
- ✅ Principle IV: Visual Learning Through Diagrams (Mermaid diagrams mandatory)
