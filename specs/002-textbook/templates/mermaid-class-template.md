# Mermaid Class Diagram Template

Use this template for showing URDF structures, data models, and entity relationships.

## Basic Structure

```mermaid
%%{title: "Class Relationships"}%%
classDiagram
    Parent "1" --> "*" Child: contains
    class Parent {
        +String name
        +method()
    }
    class Child {
        +String name
        +method()
    }
```

## URDF Robot Description Example

```mermaid
%%{title: "URDF Robot Structure"}%%
classDiagram
    Robot "1" --> "*" Link: contains
    Robot "1" --> "*" Joint: contains
    Link: +string name
    Link: +Geometry visual
    Link: +Geometry collision
    Link: +Inertial inertia
    Joint: +string name
    Joint: +string type
    Joint: +Link parent
    Joint: +Link child
    Joint: +Axis axis
```

## Best Practices

- **Relationships**: Show cardinality ("1", "*", "0..1")
- **Attributes**: Use `+` for public, `-` for private
- **Methods**: Include if relevant to understanding
- **Add title**: Always include `%%{title: "..."}%%`
- **Keep focused**: Max 6-8 classes per diagram
- **Clear labels**: Use descriptive class and attribute names
