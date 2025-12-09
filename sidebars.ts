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
  bookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Home',
    },
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'introduction/index',
        'introduction/what-is-physical-ai',
        'introduction/why-physical-ai-matters',
        'introduction/digital-to-embodied',
        'introduction/humanoid-robotics-landscape',
        'introduction/sensor-systems-overview',
      ],
    },
    {
      type: 'category',
      label: 'Getting Started',
      collapsed: false,
      items: [
        'getting-started/index',
        'getting-started/hardware-requirements',
        'getting-started/environment-setup',
        'getting-started/cloud-alternatives',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      collapsed: true,
      items: [
        'module-01-ros2/index',
        'module-01-ros2/ros2-architecture',
        'module-01-ros2/nodes-topics-services',
        'module-01-ros2/building-packages',
        'module-01-ros2/launch-files',
        'module-01-ros2/urdf-humanoids',
        'module-01-ros2/project-ros2-package',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      collapsed: true,
      items: [
        'module-02-simulation/index',
        'module-02-simulation/gazebo-fundamentals',
        'module-02-simulation/spawning-robots',
        'module-02-simulation/sensor-integration',
        'module-02-simulation/unity-robotics',
        'module-02-simulation/project-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      collapsed: true,
      items: [
        'module-03-isaac/index',
        'module-03-isaac/isaac-sim-setup',
        'module-03-isaac/vslam-visual-slam',
        'module-03-isaac/nav2-navigation',
        'module-03-isaac/isaac-ros-perception',
        'module-03-isaac/project-isaac-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Integration',
      collapsed: true,
      items: [
        'module-04-vla/index',
        'module-04-vla/whisper-integration',
        'module-04-vla/llm-planning',
        'module-04-vla/ollama-local-llms',
        'module-04-vla/action-execution',
        'module-04-vla/project-vla-pipeline',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsed: true,
      items: [
        'capstone/index',
        'capstone/system-architecture',
        'capstone/integration-guide',
        'capstone/voice-to-manipulation',
        'capstone/demo-presentation',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsed: true,
      items: [
        'appendices/index',
        'appendices/hardware-buying-guide',
        'appendices/troubleshooting',
        'appendices/learning-resources',
        'appendices/glossary',
      ],
    },
  ],
};

export default sidebars;
