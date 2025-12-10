# Mermaid Sequence Diagram Template

Use this template for showing request-response patterns, service calls, and temporal ordering.

## Basic Structure

```mermaid
%%{title: "Service Call Sequence"}%%
sequenceDiagram
    participant Client
    participant Service
    Client->>Service: Request
    Service-->>Client: Acknowledge
    Service->>Service: Process
    Service-->>Client: Response
```

## ROS 2 Service Call Example

```mermaid
%%{title: "ROS 2 Add Two Ints Service"}%%
sequenceDiagram
    participant Client as Navigation Client
    participant Service as Add Two Ints Service
    Client->>Service: Request (a=5, b=3)
    Service->>Service: Compute sum=8
    Service-->>Client: Response (sum=8)
    Client->>Client: Use result in path planning
```

## Best Practices

- **Participants**: Define key actors/systems at top
- **Arrow types**:
  - `->>` : Solid arrow (synchronous call)
  - `-->>` : Dashed arrow (async response)
- **Self-calls**: Show internal processing with `Service->>Service`
- **Add title**: Always include `%%{title: "..."}%%`
- **Keep simple**: Max 6-8 interactions
