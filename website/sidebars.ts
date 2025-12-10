import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Textbook sidebar - complete structure for all modules
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    // Module 1: ROS 2 (chapters 1-5)
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-1-nodes-architecture',
          label: 'Chapter 1: Nodes & Architecture',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-2-topics-pubsub',
          label: 'Chapter 2: Topics & Pub/Sub',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-3-services-clients',
          label: 'Chapter 3: Services & Clients',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-4-rclpy-python-client',
          label: 'Chapter 4: rclpy Python Client',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-5-urdf-robot-description',
          label: 'Chapter 5: URDF Robot Description',
        },
      ],
    },
    // Module 2: Gazebo & Unity (chapters 1-5)
    {
      type: 'category',
      label: 'Module 2: Simulation (Gazebo & Unity)',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-1-simulation-intro',
          label: 'Chapter 1: Simulation Introduction',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-2-lidar-integration',
          label: 'Chapter 2: LiDAR Integration',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-3-imu-integration',
          label: 'Chapter 3: IMU Integration',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-4-depth-camera',
          label: 'Chapter 4: Depth Camera Integration',
        },
        {
          type: 'doc',
          id: 'module-2-simulation/chapter-5-unity-environment',
          label: 'Chapter 5: Unity Environment',
        },
      ],
    },
    // Module 3: NVIDIA Isaac (chapters 1-5)
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Simulation',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-1-isaac-overview',
          label: 'Chapter 1: Isaac Overview',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-2-vslam-mapping',
          label: 'Chapter 2: Visual SLAM Mapping',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-3-nav2-navigation',
          label: 'Chapter 3: Nav2 Navigation Stack',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-4-sim-to-real-principles',
          label: 'Chapter 4: Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/chapter-5-domain-randomization',
          label: 'Chapter 5: Domain Randomization',
        },
      ],
    },
    // Module 4: VLA (chapters 1-5)
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Models',
      collapsible: true,
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'module-4-vla/chapter-1-vla-introduction',
          label: 'Chapter 1: VLA Models Introduction',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-2-vla-training',
          label: 'Chapter 2: Training VLA Models',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-3-vla-deployment',
          label: 'Chapter 3: Deploying VLA Models',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-4-vla-llm-integration',
          label: 'Chapter 4: Advanced VLA Techniques',
        },
        {
          type: 'doc',
          id: 'module-4-vla/chapter-5-vla-llm-integration',
          label: 'Chapter 5: VLA-LLM Integration',
        },
      ],
    },
  ],
};

export default sidebars;
