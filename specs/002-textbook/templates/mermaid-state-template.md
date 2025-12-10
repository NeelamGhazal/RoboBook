# Mermaid State Diagram Template

Use this template for showing robot behaviors, finite state machines, and mode transitions.

## Basic Structure

```mermaid
%%{title: "State Machine Example"}%%
stateDiagram-v2
    [*] --> Initial
    Initial --> Active: Start
    Active --> Paused: Pause
    Paused --> Active: Resume
    Active --> [*]: Stop
```

## Robot Navigation State Machine Example

```mermaid
%%{title: "Robot Navigation States"}%%
stateDiagram-v2
    [*] --> Idle
    Idle --> Navigating: Receive goal
    Navigating --> ObstacleDetected: LiDAR detects obstacle
    ObstacleDetected --> Navigating: Path clear
    ObstacleDetected --> Idle: Goal unreachable
    Navigating --> GoalReached: Position within threshold
    GoalReached --> Idle: Report success
```

## Best Practices

- **Start/End**: Use `[*]` for initial and final states
- **Transitions**: Label with triggering event or condition
- **Clear names**: Use descriptive state names (not "S1", "S2")
- **Add title**: Always include `%%{title: "..."}%%`
- **Keep manageable**: Max 8-10 states
- **Version**: Always use `stateDiagram-v2` (not v1)
