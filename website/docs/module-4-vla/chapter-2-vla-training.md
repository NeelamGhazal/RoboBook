---
title: "Training VLA Models with Demonstration Data"
sidebar_position: 18
---

# Training VLA Models with Demonstration Data

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the importance of demonstration data in VLA model training
- Describe different approaches to collecting robotic demonstration data
- Understand behavioral cloning and imitation learning techniques
- Implement data preprocessing pipelines for VLA training
- Evaluate VLA model performance on demonstration datasets

## Prerequisites

Before diving into this chapter, you should have:
- Understanding of VLA model architectures (from Chapter 1)
- Basic knowledge of machine learning and neural networks
- Experience with PyTorch or TensorFlow
- Understanding of ROS 2 for data collection
- Completion of previous modules on simulation and Isaac

## Introduction

Training Vision-Language-Action models requires high-quality demonstration data that connects visual observations, language commands, and corresponding robot actions. This demonstration data serves as the foundation for learning the complex mappings between perception, language, and action that enable robots to understand and execute tasks.

Demonstration data collection involves recording human experts performing tasks while capturing:
- Visual information from robot cameras
- Language commands or descriptions
- Robot joint angles and actions
- Environmental state information
- Task success/failure labels

The quality and diversity of demonstration data significantly impact the final performance of VLA models. Large-scale, diverse datasets enable models to generalize across different objects, environments, and tasks.

## Theory

### Behavioral Cloning

Behavioral cloning is the most common approach for training VLA models on demonstration data. It treats the problem as supervised learning, where the model learns to map observations (vision + language) to actions by mimicking demonstrated behaviors.

The loss function typically minimizes the difference between predicted actions and demonstrated actions:

L = ||π_θ(s) - a_demo||²

Where π_θ is the policy network with parameters θ, s is the state (vision + language), and a_demo is the demonstrated action.

### Imitation Learning

Beyond simple behavioral cloning, imitation learning encompasses various techniques:
- **DAgger (Dataset Aggregation)**: Addresses distribution shift by iteratively collecting new data from the policy
- **Adversarial Imitation Learning**: Uses adversarial training to match the expert's behavior distribution
- **Offline Reinforcement Learning**: Fine-tunes behavioral cloning with RL objectives

### Data Collection Strategies

Effective VLA training requires diverse and high-quality demonstration data:

1. **Human Demonstrations**: Recording human operators controlling robots
2. **Teleoperation**: Using haptic interfaces or joysticks for precise control
3. **Kinesthetic Teaching**: Physically guiding the robot through motions
4. **Virtual Reality**: Using VR interfaces for intuitive demonstration
5. **Simulation-to-Real**: Collecting data in simulation and transferring to real robots

### Dataset Requirements

High-quality VLA datasets should include:
- Diverse object categories and arrangements
- Various lighting conditions and backgrounds
- Multiple language expressions for the same task
- Different robot configurations and starting positions
- Failure cases and recovery behaviors
- Long-horizon tasks with temporal dependencies

## Code Example

Here's an example of how to collect and preprocess demonstration data for VLA model training:

```python
import torch
import torch.nn as nn
import numpy as np
import cv2
import json
import os
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from transformers import CLIPTokenizer

class VLADataset(Dataset):
    """
    Dataset class for VLA demonstration data.
    Assumes data is stored in JSON format with image paths, commands, and actions.
    """

    def __init__(self, data_dir, transform=None):
        """
        Initialize the VLA dataset.

        Args:
            data_dir: Directory containing demonstration data
            transform: Image transformations to apply
        """
        self.data_dir = data_dir
        self.transform = transform or transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((224, 224)),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # Load demonstration data
        self.demonstrations = []
        self._load_demonstrations()

    def _load_demonstrations(self):
        """Load demonstration data from JSON files."""
        for filename in os.listdir(self.data_dir):
            if filename.endswith('.json'):
                with open(os.path.join(self.data_dir, filename), 'r') as f:
                    demo_data = json.load(f)
                    self.demonstrations.extend(demo_data['episodes'])

    def __len__(self):
        return len(self.demonstrations)

    def __getitem__(self, idx):
        """
        Get a single demonstration sample.

        Args:
            idx: Index of the demonstration

        Returns:
            Dictionary containing image, command, and action
        """
        episode = self.demonstrations[idx]

        # Load image
        image_path = os.path.join(self.data_dir, episode['image_path'])
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if self.transform:
            image = self.transform(image)

        # Process command
        command = episode['command']

        # Process action
        action = torch.tensor(episode['action'], dtype=torch.float32)

        return {
            'image': image,
            'command': command,
            'action': action
        }

class VLADataCollector:
    """
    Class for collecting VLA demonstration data from ROS 2 topics.
    """

    def __init__(self, save_dir="vla_demonstrations"):
        self.save_dir = save_dir
        self.episode_data = []
        self.current_episode = []

        # Create save directory
        os.makedirs(save_dir, exist_ok=True)

        # Initialize ROS 2 subscriptions
        # In practice, you would subscribe to camera, joint state, and command topics
        # self.camera_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        # self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        # self.command_sub = rospy.Subscriber('/command', String, self.command_callback)

    def start_episode(self):
        """Start a new demonstration episode."""
        self.current_episode = []
        print("Starting new demonstration episode...")

    def record_step(self, image, joints, command, action):
        """
        Record a single step of the demonstration.

        Args:
            image: Current camera image
            joints: Current joint states
            command: Language command
            action: Action taken (next joint positions or velocity)
        """
        step_data = {
            'image_path': f"step_{len(self.current_episode)}.jpg",
            'joints': joints.tolist() if isinstance(joints, np.ndarray) else joints,
            'command': command,
            'action': action.tolist() if isinstance(action, np.ndarray) else action,
            'timestamp': len(self.current_episode)  # Relative timestamp within episode
        }

        # Save image
        image_filename = os.path.join(self.save_dir, step_data['image_path'])
        cv2.imwrite(image_filename, image)

        self.current_episode.append(step_data)

    def end_episode(self, success=True):
        """End the current demonstration episode."""
        episode_data = {
            'steps': self.current_episode,
            'success': success,
            'length': len(self.current_episode)
        }

        # Save episode data
        episode_id = len(self.episode_data)
        episode_filename = os.path.join(self.save_dir, f"episode_{episode_id}.json")

        with open(episode_filename, 'w') as f:
            json.dump(episode_data, f, indent=2)

        self.episode_data.append(episode_data)
        print(f"Episode saved: {episode_filename}")

    def collect_demonstration(self, task_description):
        """
        Collect a full demonstration for a specific task.

        Args:
            task_description: Natural language description of the task
        """
        print(f"Collecting demonstration for: {task_description}")

        self.start_episode()

        # In practice, this would involve:
        # 1. Waiting for user to start task execution
        # 2. Continuously recording state-action pairs
        # 3. Detecting task completion
        # 4. Saving the complete episode

        # Simulate some steps for demonstration
        for i in range(10):  # Simulate 10 steps
            # Simulated data (in practice, this comes from ROS 2 topics)
            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            dummy_joints = np.random.rand(7).astype(np.float32)
            dummy_action = np.random.rand(7).astype(np.float32)

            self.record_step(dummy_image, dummy_joints, task_description, dummy_action)

        self.end_episode(success=True)

def train_vla_model(model, dataset, epochs=10, batch_size=32, lr=1e-4):
    """
    Train a VLA model using behavioral cloning.

    Args:
        model: VLA model to train
        dataset: VLA dataset
        epochs: Number of training epochs
        batch_size: Training batch size
        lr: Learning rate
    """
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    model.train()

    for epoch in range(epochs):
        total_loss = 0.0

        for batch in dataloader:
            images = batch['image']
            commands = batch['command']
            actions = batch['action']

            optimizer.zero_grad()

            # Forward pass
            predicted_actions = model(images, commands)

            # Calculate loss
            loss = criterion(predicted_actions, actions)

            # Backward pass
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1}/{epochs}, Loss: {avg_loss:.4f}")

# Example usage
def example_usage():
    """
    Example of how to use the VLA data collection and training pipeline.
    """
    # Initialize data collector
    collector = VLADataCollector()

    # Collect demonstrations for different tasks
    tasks = [
        "Pick up the red cup and place it on the table",
        "Move the blue box to the left side",
        "Open the drawer and take out the object"
    ]

    for task in tasks:
        collector.collect_demonstration(task)

    # Create dataset from collected demonstrations
    dataset = VLADataset(collector.save_dir)

    # Initialize model (assuming we have a VLA model class)
    # model = VLAModel(action_dim=7)  # 7-DOF robot arm

    # Train the model
    # train_vla_model(model, dataset)

    print(f"Collected {len(dataset)} demonstration samples")

if __name__ == "__main__":
    example_usage()
```

This code example demonstrates a complete pipeline for collecting demonstration data for VLA models and training them using behavioral cloning. The code includes data collection utilities, dataset classes, and training functions.

## Exercises

1. **Data Collection Enhancement**:
   - Extend the VLADataCollector class to support multiple camera views
   - Add support for different sensor modalities (LiDAR, IMU, etc.)
   - Implement automatic episode segmentation based on motion detection

2. **Advanced Training Techniques**:
   - Implement DAgger algorithm for iterative improvement
   - Add data augmentation techniques for vision and language inputs
   - Implement curriculum learning for progressive skill acquisition

3. **Evaluation Metrics**:
   - Design evaluation metrics for VLA model performance
   - Implement success rate calculation for different task types
   - Create visualization tools for analyzing model predictions vs. ground truth

## Summary

Training VLA models with demonstration data requires careful consideration of data collection, preprocessing, and training methodologies. Behavioral cloning provides a solid foundation for learning from expert demonstrations, while more advanced techniques like DAgger and adversarial imitation learning can address distribution shift and improve performance.

The quality and diversity of demonstration data significantly impact the final model performance. Effective data collection strategies should capture various scenarios, objects, and environmental conditions to enable robust generalization.

## Next Steps

In the next chapter, we'll explore how to deploy VLA models on real robots using ROS 2, including inference optimization, safety considerations, and integration with existing robotic systems.