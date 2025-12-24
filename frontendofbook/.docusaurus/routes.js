import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/Root',
    component: ComponentCreator('/Root', '940'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '732'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '170'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'bd5'),
            routes: [
              {
                path: '/docs/ai-integration/overview',
                component: ComponentCreator('/docs/ai-integration/overview', 'ae4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ai-integration/rag-system',
                component: ComponentCreator('/docs/ai-integration/rag-system', '8b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/conclusion',
                component: ComponentCreator('/docs/conclusion', '95e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/development/spec-driven-approach',
                component: ComponentCreator('/docs/development/spec-driven-approach', 'a98'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/development/workflow',
                component: ComponentCreator('/docs/development/workflow', '3da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/digital-twins-hri-unity',
                component: ComponentCreator('/docs/digital-twin/digital-twins-hri-unity', '4ae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/physics-simulation-gazebo',
                component: ComponentCreator('/docs/digital-twin/physics-simulation-gazebo', 'f35'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin/sensor-simulation-validation',
                component: ComponentCreator('/docs/digital-twin/sensor-simulation-validation', '737'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/getting-started/configuration',
                component: ComponentCreator('/docs/getting-started/configuration', '468'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/getting-started/installation',
                component: ComponentCreator('/docs/getting-started/installation', '267'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/isaac-ros-vslam-navigation',
                component: ComponentCreator('/docs/isaac-ai-brain/isaac-ros-vslam-navigation', '0c5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/isaac-sim-photorealistic-simulation',
                component: ComponentCreator('/docs/isaac-ai-brain/isaac-sim-photorealistic-simulation', '3e2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-ai-brain/nav2-path-planning-humanoid',
                component: ComponentCreator('/docs/isaac-ai-brain/nav2-path-planning-humanoid', 'fdd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-humanoid/communication-model',
                component: ComponentCreator('/docs/ros2-humanoid/communication-model', 'fd0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-humanoid/intro-to-ros2',
                component: ComponentCreator('/docs/ros2-humanoid/intro-to-ros2', '96c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-humanoid/robot-structure-urdf',
                component: ComponentCreator('/docs/ros2-humanoid/robot-structure-urdf', '8bf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-llm-integration/capstone-autonomous-humanoid-tasks',
                component: ComponentCreator('/docs/vla-llm-integration/capstone-autonomous-humanoid-tasks', '3d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-llm-integration/cognitive-planning-llms-ros2',
                component: ComponentCreator('/docs/vla-llm-integration/cognitive-planning-llms-ros2', '237'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-llm-integration/voice-to-action-whisper',
                component: ComponentCreator('/docs/vla-llm-integration/voice-to-action-whisper', '699'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
