// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started/installation', 'getting-started/configuration'],
    },
    {
      type: 'category',
      label: 'AI Integration',
      items: ['ai-integration/overview', 'ai-integration/rag-system'],
    },
    {
      type: 'category',
      label: 'Module-01 (ROS-2 For Humanoid Robotics)',
      items: [
        'ros2-humanoid/intro-to-ros2',
        'ros2-humanoid/communication-model',
        'ros2-humanoid/robot-structure-urdf'
      ],
    },
    {
      type: 'category',
      label: 'Module-02 (Digital Twin for Humanoid Robotics)',
      items: [
        'digital-twin/physics-simulation-gazebo',
        'digital-twin/digital-twins-hri-unity',
        'digital-twin/sensor-simulation-validation'
      ],
    },
    {
      type: 'category',
      label: 'Module-03 (Isaac AI Brain for Humanoid Robotics)',
      items: [
        'isaac-ai-brain/isaac-sim-photorealistic-simulation',
        'isaac-ai-brain/isaac-ros-vslam-navigation',
        'isaac-ai-brain/nav2-path-planning-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Module-04 (VLA for Humanoid Robotics)',
      items: [
        'vla-llm-integration/voice-to-action-whisper',
        'vla-llm-integration/cognitive-planning-llms-ros2',
        'vla-llm-integration/capstone-autonomous-humanoid-tasks'
      ],
    },
    {
      type: 'category',
      label: 'Development',
      items: ['development/workflow', 'development/spec-driven-approach'],
    },
    'conclusion',
  ],
};

module.exports = sidebars;