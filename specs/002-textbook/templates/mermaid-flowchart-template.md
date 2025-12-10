# Mermaid Flowchart Template

Use this template for showing data flow, node communication, and system architecture.

## Basic Structure

```mermaid
%%{title: "Descriptive Title for Accessibility"}%%
graph LR
    A[Start Node] -->|labeled edge| B[Process Node]
    B --> C[Decision Node]
    C -->|yes| D[End Node 1]
    C -->|no| E[End Node 2]
```

## ROS 2 Node Communication Example

```mermaid
%%{title: "ROS 2 Publisher-Subscriber Pattern"}%%
graph LR
    A[Publisher Node] -->|topic: /sensor_data| B[ROS 2 Middleware DDS]
    B --> C[Subscriber Node 1]
    B --> D[Subscriber Node 2]
    C --> E[Process Data]
    D --> F[Log Data]
```

## Best Practices

- **Max 10-12 nodes**: Keep diagrams simple
- **Clear labels**: Use descriptive names (not "node1", "node2")
- **Add title**: Always include `%%{title: "..."}%%` for accessibility
- **Direction**: Use `graph LR` (left-to-right) or `graph TD` (top-to-down)
- **Test rendering**: Verify in Docusaurus build before committing
