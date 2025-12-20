# Cognitive Planning using LLMs for ROS 2

## Introduction

This chapter explores the use of Large Language Models (LLMs) for cognitive planning in robotics, specifically focusing on converting natural language commands into ROS 2 action sequences. Cognitive planning bridges the gap between high-level human instructions and low-level robotic actions, enabling natural human-robot interaction.

The chapter will cover prompt engineering techniques for LLMs, mapping natural language to ROS 2 actions, implementing safety checks, and creating robust cognitive planning systems. You'll learn how to design LLM-based planners that can interpret complex commands and generate appropriate robot behaviors.

## Learning Objectives

By the end of this chapter, you will be able to:
- Design effective prompts for LLM-based cognitive planning
- Map natural language commands to ROS 2 action sequences
- Implement safety checks and validation in cognitive planning
- Create context-aware planning systems for multi-step tasks
- Optimize LLM performance for real-time robotic applications

## Prerequisites

Before starting this chapter, you should have:
- Completed the voice-to-action chapter (Module 4, Chapter 1)
- Understanding of ROS 2 action concepts and message types
- Basic knowledge of LLM APIs (OpenAI, Anthropic, etc.)
- Familiarity with natural language processing concepts

## Understanding Cognitive Planning in Robotics

### The Cognitive Planning Problem

Cognitive planning in robotics involves translating high-level goals or natural language commands into executable action sequences. This process requires:

1. **Understanding**: Interpreting the user's intent from natural language
2. **Reasoning**: Determining the appropriate sequence of actions
3. **Planning**: Creating a detailed plan of ROS 2 actions
4. **Execution**: Coordinating the execution of the planned actions
5. **Monitoring**: Tracking execution and adapting as needed

### LLM Capabilities for Cognitive Planning

Large Language Models excel at cognitive planning tasks because they can:

- Understand complex natural language commands
- Reason about object relationships and spatial concepts
- Generate structured outputs (JSON, XML) that can be parsed
- Handle multi-step reasoning and planning
- Adapt to new situations through few-shot learning

### Challenges in LLM-Based Planning

However, LLMs also present challenges for robotic applications:

- **Non-deterministic outputs**: LLMs may generate inconsistent responses
- **Lack of real-time constraints**: LLMs may not consider timing constraints
- **Safety concerns**: LLMs may generate unsafe commands without proper validation
- **Context limitations**: LLMs have finite context windows
- **Latency**: LLM calls may introduce significant delays

## LLM Selection and Setup

### Popular LLM Options for Robotics

Different LLMs offer various trade-offs for cognitive planning:

| LLM | Strengths | Weaknesses | Use Case |
|-----|-----------|------------|----------|
| GPT-4 | High accuracy, good reasoning | Expensive, slower | Complex planning tasks |
| GPT-3.5 | Good balance, faster | Less reasoning capability | Real-time applications |
| Claude | Good reasoning, longer context | Newer, limited access | Complex multi-step planning |
| Local models (Llama, etc.) | Private, controllable | Requires significant resources | Privacy-sensitive applications |

### Basic LLM Integration

```python
# Example: Basic LLM integration for cognitive planning
import openai
import json
import os
from typing import Dict, List, Any

class LLMBridge:
    def __init__(self, api_key: str = None, model: str = "gpt-3.5-turbo"):
        if api_key:
            openai.api_key = api_key
        else:
            openai.api_key = os.getenv("OPENAI_API_KEY")

        self.model = model

    def call_llm(self, prompt: str, max_tokens: int = 500, temperature: float = 0.3) -> str:
        """
        Call the LLM with a given prompt
        """
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=max_tokens,
            temperature=temperature,
            response_format={"type": "json_object"}  # For structured output
        )

        return response.choices[0].message['content'].strip()

    def validate_response(self, response: str) -> Dict[str, Any]:
        """
        Validate and parse LLM response
        """
        try:
            parsed = json.loads(response)
            return parsed
        except json.JSONDecodeError:
            # If JSON parsing fails, try to extract structured data using regex
            # This is a fallback approach
            return {"error": "Invalid JSON response", "raw_response": response}
```

## Prompt Engineering for Cognitive Planning

### Effective Prompt Structure

Well-designed prompts are crucial for reliable cognitive planning:

```python
# Example: Structured prompt for cognitive planning
class CognitivePlanningPrompter:
    def __init__(self):
        self.system_prompt = """
        You are a cognitive planning assistant for a humanoid robot. Your task is to interpret natural language commands and convert them into structured action sequences for ROS 2 execution.

        Guidelines:
        1. Always prioritize safety - never generate commands that could harm humans or damage property
        2. Break complex commands into simple, executable steps
        3. Use only the action types provided in the action vocabulary
        4. Include appropriate parameters for each action
        5. Consider the robot's current state and environment when planning

        Action Vocabulary:
        - navigation: move to a location, parameters: [x, y, theta, frame_id]
        - manipulation: manipulate objects, parameters: [object_name, action_type, grasp_pose]
        - perception: perceive environment, parameters: [sensor_type, target_object]
        - communication: communicate with humans, parameters: [message_type, content]

        Output Format:
        {
            "command": "original user command",
            "intent": "parsed intent",
            "actions": [
                {
                    "action_type": "navigation|manipulation|perception|communication",
                    "parameters": {"param1": "value1", ...},
                    "description": "human-readable description"
                }
            ],
            "confidence": 0.0-1.0,
            "safety_check_passed": true/false
        }
        """

    def create_planning_prompt(self, user_command: str, robot_state: Dict = None,
                              environment_info: Dict = None) -> str:
        """
        Create a structured prompt for cognitive planning
        """
        prompt = f"{self.system_prompt}\n\n"

        if robot_state:
            prompt += f"Current Robot State: {json.dumps(robot_state)}\n\n"

        if environment_info:
            prompt += f"Environment Information: {json.dumps(environment_info)}\n\n"

        prompt += f"User Command: {user_command}\n\n"
        prompt += "Please generate the appropriate action sequence:"

        return prompt

    def create_validation_prompt(self, action_sequence: List[Dict], user_command: str) -> str:
        """
        Create a prompt to validate an action sequence
        """
        validation_prompt = f"""
        You are validating a robot action sequence for safety and correctness.

        Original Command: {user_command}

        Action Sequence: {json.dumps(action_sequence, indent=2)}

        Please validate this sequence and respond in JSON format:
        {{
            "is_safe": true/false,
            "is_complete": true/false,
            "issues": ["list of issues if any"],
            "suggestions": ["list of suggestions if needed"]
        }}
        """

        return validation_prompt
```

### Advanced Prompt Techniques

```python
# Example: Advanced prompt engineering techniques
class AdvancedPrompter:
    def __init__(self):
        self.action_templates = {
            "navigation": {
                "examples": [
                    {
                        "command": "Go to the kitchen",
                        "actions": [{"action_type": "navigation", "parameters": {"x": 2.5, "y": 1.0, "theta": 0.0, "frame_id": "map"}}]
                    },
                    {
                        "command": "Move 2 meters forward",
                        "actions": [{"action_type": "navigation", "parameters": {"x": 2.0, "y": 0.0, "theta": 0.0, "frame_id": "base_link"}}]
                    }
                ]
            },
            "manipulation": {
                "examples": [
                    {
                        "command": "Pick up the red cup",
                        "actions": [{"action_type": "perception", "parameters": {"sensor_type": "camera", "target_object": "red cup"}},
                                   {"action_type": "manipulation", "parameters": {"object_name": "red cup", "action_type": "grasp", "grasp_pose": {"x": 0.5, "y": 0.2, "z": 0.1}}}]
                    }
                ]
            }
        }

    def few_shot_prompt(self, user_command: str, command_type: str) -> str:
        """
        Create a few-shot learning prompt with examples
        """
        examples = self.action_templates.get(command_type, {}).get("examples", [])

        prompt = f"""
        You are a cognitive planning assistant. Here are examples of how to convert natural language commands to action sequences:

        Examples:
        """

        for i, example in enumerate(examples[:2]):  # Use first 2 examples
            prompt += f"\nExample {i+1}:"
            prompt += f"\nCommand: {example['command']}"
            prompt += f"\nActions: {json.dumps(example['actions'], indent=2)}"
            prompt += "\n---"

        prompt += f"\n\nNow process this command: {user_command}"
        prompt += "\n\nRespond with the appropriate action sequence in JSON format."

        return prompt

    def chain_of_thought_prompt(self, user_command: str) -> str:
        """
        Create a chain-of-thought prompt that explains the reasoning
        """
        prompt = f"""
        You are a cognitive planning assistant. For the following command, think step by step:

        Command: {user_command}

        Step-by-step reasoning:
        1. What is the user trying to achieve?
        2. What are the key objects or locations involved?
        3. What sequence of actions is needed?
        4. What parameters are required for each action?

        Finally, provide the action sequence in JSON format with the following structure:
        {{
            "command": "{user_command}",
            "reasoning": "step-by-step explanation",
            "actions": [
                {{
                    "action_type": "navigation|manipulation|perception|communication",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "what this action does"
                }}
            ]
        }}
        """

        return prompt
```

## Mapping Natural Language to ROS 2 Actions

### ROS 2 Action Structure

Understanding how to map LLM outputs to ROS 2 actions:

```python
# Example: ROS 2 action mapping system
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

class ROS2ActionMapper(Node):
    def __init__(self):
        super().__init__('ros2_action_mapper')

        # Action clients for different robot capabilities
        self.navigation_client = ActionClient(self, 'nav2_msgs/action/NavigateToPose', 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, 'manipulation_msgs/action/Grasp', 'grasp_object')

        # Publishers for other actions
        self.cmd_vel_publisher = self.create_publisher('geometry_msgs/msg/Twist', 'cmd_vel', 10)
        self.speech_publisher = self.create_publisher('std_msgs/msg/String', 'robot_speech', 10)

    def execute_action_sequence(self, action_sequence: List[Dict]) -> bool:
        """
        Execute a sequence of actions mapped from LLM output
        """
        success = True

        for action in action_sequence:
            action_type = action.get('action_type')
            parameters = action.get('parameters', {})

            try:
                if action_type == 'navigation':
                    success &= self.execute_navigation_action(parameters)
                elif action_type == 'manipulation':
                    success &= self.execute_manipulation_action(parameters)
                elif action_type == 'perception':
                    success &= self.execute_perception_action(parameters)
                elif action_type == 'communication':
                    success &= self.execute_communication_action(parameters)
                else:
                    self.get_logger().warn(f"Unknown action type: {action_type}")
                    success = False

            except Exception as e:
                self.get_logger().error(f"Error executing action {action_type}: {e}")
                success = False

        return success

    def execute_navigation_action(self, params: Dict) -> bool:
        """
        Execute navigation action
        """
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = params.get('frame_id', 'map')
        goal_msg.pose.pose.position.x = params.get('x', 0.0)
        goal_msg.pose.pose.position.y = params.get('y', 0.0)

        # Set orientation
        import math
        theta = params.get('theta', 0.0)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send goal
        self.navigation_client.wait_for_server()
        future = self.navigation_client.send_goal_async(goal_msg)

        # Wait for result (in a real system, you'd handle this asynchronously)
        # rclpy.spin_until_future_complete(self, future)

        return True  # Simplified for example

    def execute_manipulation_action(self, params: Dict) -> bool:
        """
        Execute manipulation action
        """
        # Implementation would depend on your specific manipulation stack
        object_name = params.get('object_name')
        action_type = params.get('action_type')

        self.get_logger().info(f"Manipulation: {action_type} {object_name}")
        return True

    def execute_perception_action(self, params: Dict) -> bool:
        """
        Execute perception action
        """
        sensor_type = params.get('sensor_type')
        target_object = params.get('target_object')

        self.get_logger().info(f"Perception: {sensor_type} for {target_object}")
        return True

    def execute_communication_action(self, params: Dict) -> bool:
        """
        Execute communication action
        """
        message_type = params.get('message_type')
        content = params.get('content')

        msg = String()
        msg.data = content
        self.speech_publisher.publish(msg)

        return True
```

### Safety and Validation Layer

Implementing safety checks for LLM-generated actions:

```python
# Example: Safety and validation layer
class SafetyValidator:
    def __init__(self):
        self.safety_rules = [
            self.no_self_harm,
            self.no_harm_to_others,
            self.no_property_damage,
            self.no_impossible_actions,
            self.no_privacy_violations
        ]

        # Known dangerous commands
        self.dangerous_keywords = [
            'self-destruct', 'harm', 'destroy', 'damage', 'kill', 'attack',
            'violate', 'break', 'ignore safety', 'emergency stop'
        ]

    def validate_action_sequence(self, action_sequence: List[Dict],
                               original_command: str = "") -> Dict[str, Any]:
        """
        Validate an action sequence for safety
        """
        validation_result = {
            'is_safe': True,
            'issues': [],
            'suggestions': []
        }

        # Check for dangerous keywords in original command
        for keyword in self.dangerous_keywords:
            if keyword.lower() in original_command.lower():
                validation_result['is_safe'] = False
                validation_result['issues'].append(f"Dangerous keyword detected: {keyword}")
                return validation_result

        # Validate each action
        for i, action in enumerate(action_sequence):
            action_validation = self.validate_single_action(action, i)

            if not action_validation['is_safe']:
                validation_result['is_safe'] = False
                validation_result['issues'].extend(action_validation['issues'])

        return validation_result

    def validate_single_action(self, action: Dict, index: int) -> Dict[str, Any]:
        """
        Validate a single action for safety
        """
        result = {'is_safe': True, 'issues': [], 'suggestions': []}

        action_type = action.get('action_type', '')
        params = action.get('parameters', {})

        # Apply safety rules
        for rule in self.safety_rules:
            rule_result = rule(action_type, params, index)
            if not rule_result['is_safe']:
                result['is_safe'] = False
                result['issues'].extend(rule_result['issues'])

        return result

    def no_self_harm(self, action_type: str, params: Dict, index: int) -> Dict[str, Any]:
        """
        Rule: Prevent actions that could harm the robot
        """
        issues = []

        # Check for commands that might damage the robot
        if action_type == 'navigation':
            # Check for extreme velocities
            vel_x = params.get('linear_velocity', 0)
            if abs(vel_x) > 2.0:  # Adjust threshold as needed
                issues.append(f"Navigation action {index}: Excessive linear velocity ({vel_x})")

        return {'is_safe': len(issues) == 0, 'issues': issues}

    def no_harm_to_others(self, action_type: str, params: Dict, index: int) -> Dict[str, Any]:
        """
        Rule: Prevent actions that could harm humans
        """
        issues = []

        # Check for potentially dangerous navigation
        if action_type == 'navigation':
            # Check for navigation toward humans (simplified check)
            target_x = params.get('x', 0)
            target_y = params.get('y', 0)
            target_distance = (target_x**2 + target_y**2)**0.5

            if target_distance < 0.5:  # Too close to target
                issues.append(f"Navigation action {index}: Target too close ({target_distance:.2f}m)")

        return {'is_safe': len(issues) == 0, 'issues': issues}

    def no_impossible_actions(self, action_type: str, params: Dict, index: int) -> Dict[str, Any]:
        """
        Rule: Prevent physically impossible actions
        """
        issues = []

        # Check for impossible manipulation
        if action_type == 'manipulation':
            object_weight = params.get('object_weight', 0)
            if object_weight > 10.0:  # Assuming robot can't lift more than 10kg
                issues.append(f"Manipulation action {index}: Object too heavy ({object_weight}kg)")

        return {'is_safe': len(issues) == 0, 'issues': issues}
```

## Context Management for Multi-Step Tasks

### Maintaining Conversation Context

Handling multi-step tasks that require maintaining context:

```python
# Example: Context management for multi-step tasks
class ContextManager:
    def __init__(self):
        self.conversation_history = []
        self.robot_state = {}
        self.environment_map = {}
        self.object_locations = {}
        self.task_context = {}

    def update_context(self, user_command: str, action_sequence: List[Dict],
                      execution_result: Dict = None):
        """
        Update the context with new information
        """
        # Add to conversation history
        self.conversation_history.append({
            'user_command': user_command,
            'action_sequence': action_sequence,
            'execution_result': execution_result,
            'timestamp': time.time()
        })

        # Keep only recent history to manage context window
        if len(self.conversation_history) > 10:  # Keep last 10 interactions
            self.conversation_history = self.conversation_history[-10:]

    def get_context_prompt(self) -> str:
        """
        Get a prompt with relevant context information
        """
        context_prompt = "Context Information:\n"

        # Add recent conversation history
        if self.conversation_history:
            context_prompt += "Recent interactions:\n"
            for interaction in self.conversation_history[-3:]:  # Last 3 interactions
                context_prompt += f"- User: {interaction['user_command']}\n"
                if interaction['execution_result']:
                    context_prompt += f"  Result: {interaction['execution_result'].get('status', 'unknown')}\n"

        # Add current robot state
        if self.robot_state:
            context_prompt += f"Robot state: {json.dumps(self.robot_state)}\n"

        # Add known object locations
        if self.object_locations:
            context_prompt += f"Known objects: {json.dumps(self.object_locations)}\n"

        return context_prompt

    def resolve_pronouns(self, command: str) -> str:
        """
        Resolve pronouns in the command based on context
        """
        # Simple pronoun resolution based on context
        if "it" in command.lower() and self.object_locations:
            # Replace "it" with the most recently mentioned object
            last_object = list(self.object_locations.keys())[-1] if self.object_locations else None
            if last_object:
                command = command.replace(" it ", f" {last_object} ")

        if "there" in command.lower() and self.conversation_history:
            # Replace "there" with the last mentioned location
            last_action = self.conversation_history[-1].get('action_sequence', [{}])[-1]
            if last_action.get('action_type') == 'navigation':
                params = last_action.get('parameters', {})
                location = f"{params.get('x', 0)}, {params.get('y', 0)}"
                command = command.replace(" there ", f" to {location} ")

        return command
```

### Task Decomposition and Planning

Breaking down complex tasks into manageable steps:

```python
# Example: Task decomposition and hierarchical planning
class TaskDecomposer:
    def __init__(self):
        self.task_library = {
            "fetch_object": ["locate_object", "navigate_to_object", "grasp_object", "return_with_object"],
            "room_cleaning": ["survey_room", "identify_objects", "categorize_objects", "move_objects"],
            "guided_tour": ["welcome_user", "navigate_to_point", "provide_information", "proceed_to_next_point"]
        }

    def decompose_task(self, high_level_command: str) -> List[str]:
        """
        Decompose a high-level command into subtasks
        """
        # Check if command matches a known task pattern
        command_lower = high_level_command.lower()

        for task_name, subtasks in self.task_library.items():
            if task_name in command_lower:
                return subtasks

        # If no known pattern, return the command as a single task
        return [high_level_command]

    def create_hierarchical_plan(self, high_level_command: str,
                               llm_bridge: LLMBridge) -> List[Dict]:
        """
        Create a hierarchical plan for complex tasks
        """
        subtasks = self.decompose_task(high_level_command)
        hierarchical_plan = []

        for subtask in subtasks:
            # Generate detailed plan for each subtask
            prompt = f"Create a detailed action plan for: {subtask}. " \
                    f"Original high-level command: {high_level_command}. " \
                    "Respond in JSON format with the action sequence."

            response = llm_bridge.call_llm(prompt)
            subtask_plan = llm_bridge.validate_response(response)

            if "actions" in subtask_plan:
                hierarchical_plan.append({
                    "subtask": subtask,
                    "actions": subtask_plan["actions"],
                    "description": subtask_plan.get("intent", subtask)
                })
            else:
                hierarchical_plan.append({
                    "subtask": subtask,
                    "actions": [{"action_type": "communication",
                               "parameters": {"message_type": "error",
                                            "content": f"Could not plan subtask: {subtask}"}}],
                    "description": f"Failed to plan subtask: {subtask}"
                })

        return hierarchical_plan
```

## Integration with ROS 2 Ecosystem

### ROS 2 Node Implementation

Creating a complete ROS 2 node for cognitive planning:

```python
# Example: Complete ROS 2 cognitive planning node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import asyncio
import threading
import time

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Initialize components
        self.llm_bridge = LLMBridge()
        self.prompter = CognitivePlanningPrompter()
        self.action_mapper = ROS2ActionMapper()
        self.safety_validator = SafetyValidator()
        self.context_manager = ContextManager()
        self.task_decomposer = TaskDecomposer()

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String, 'voice_commands', self.voice_command_callback, 10
        )
        self.text_command_sub = self.create_subscription(
            String, 'text_commands', self.text_command_callback, 10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, 'cognitive_planner_status', 10)
        self.action_sequence_pub = self.create_publisher(String, 'action_sequence', 10)

        # Service clients for environment information
        self.get_map_client = self.create_client(GetMap, 'map_server/map')

        self.get_logger().info('Cognitive Planner Node initialized')

    def voice_command_callback(self, msg):
        """
        Handle voice command input
        """
        self.process_command(msg.data, command_type='voice')

    def text_command_callback(self, msg):
        """
        Handle text command input
        """
        self.process_command(msg.data, command_type='text')

    def process_command(self, command: str, command_type: str = 'text'):
        """
        Process a natural language command through the cognitive planning pipeline
        """
        try:
            # Resolve pronouns and context
            resolved_command = self.context_manager.resolve_pronouns(command)

            # Get environment information
            env_info = self.get_environment_info()

            # Create planning prompt with context
            context_prompt = self.context_manager.get_context_prompt()
            full_prompt = self.prompter.create_planning_prompt(
                resolved_command,
                robot_state=self.get_robot_state(),
                environment_info=env_info
            )

            # Call LLM to generate action sequence
            llm_response = self.llm_bridge.call_llm(full_prompt)
            action_sequence = self.llm_bridge.validate_response(llm_response)

            # Validate for safety
            validation_result = self.safety_validator.validate_action_sequence(
                action_sequence.get('actions', []), resolved_command
            )

            if not validation_result['is_safe']:
                self.get_logger().error(f"Unsafe action sequence rejected: {validation_result['issues']}")
                self.publish_status(f"Command rejected for safety reasons: {validation_result['issues']}")
                return

            # Publish action sequence for execution
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_sequence_pub.publish(action_msg)

            # Update context with this interaction
            self.context_manager.update_context(resolved_command, action_sequence)

            self.get_logger().info(f"Generated action sequence for: {resolved_command}")
            self.publish_status(f"Planning complete for: {resolved_command}")

        except Exception as e:
            self.get_logger().error(f"Error processing command '{command}': {e}")
            self.publish_status(f"Error processing command: {e}")

    def get_environment_info(self) -> Dict:
        """
        Get current environment information
        """
        env_info = {}

        # This would integrate with perception systems to get:
        # - Object locations
        # - Obstacle maps
        # - Human locations
        # - Room layout

        # For now, return a simple representation
        return {
            "object_locations": self.context_manager.object_locations,
            "known_rooms": ["kitchen", "living_room", "bedroom"],
            "robot_location": self.get_robot_position()
        }

    def get_robot_state(self) -> Dict:
        """
        Get current robot state
        """
        # This would integrate with robot state publisher
        return {
            "battery_level": 0.85,
            "current_pose": self.get_robot_position(),
            "manipulator_status": "ready",
            "navigation_status": "idle"
        }

    def get_robot_position(self) -> Dict:
        """
        Get robot's current position (simplified)
        """
        # In a real system, this would come from TF or localization
        return {"x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "map"}

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
```

## Performance Optimization

### Caching and Optimization Strategies

Optimizing LLM-based cognitive planning for performance:

```python
# Example: Performance optimization strategies
import functools
import hashlib
from typing import Optional

class OptimizedCognitivePlanner:
    def __init__(self, llm_bridge: LLMBridge):
        self.llm_bridge = llm_bridge
        self.response_cache = {}
        self.cache_size_limit = 100

    def get_cache_key(self, prompt: str, params: Dict) -> str:
        """
        Generate a cache key for the prompt and parameters
        """
        cache_input = f"{prompt}_{json.dumps(params, sort_keys=True)}"
        return hashlib.md5(cache_input.encode()).hexdigest()

    @functools.lru_cache(maxsize=50)
    def cached_llm_call(self, prompt: str, max_tokens: int = 500) -> str:
        """
        Cached LLM call for frequently used prompts
        """
        return self.llm_bridge.call_llm(prompt, max_tokens=max_tokens)

    def intelligent_caching(self, command: str, action_sequence: List[Dict]):
        """
        Implement intelligent caching based on command similarity
        """
        # Cache simple, frequently used commands
        if len(action_sequence) <= 3:  # Simple commands
            cache_key = hashlib.md5(command.encode()).hexdigest()
            self.response_cache[cache_key] = action_sequence

            # Limit cache size
            if len(self.response_cache) > self.cache_size_limit:
                # Remove oldest entries (simplified)
                oldest_key = next(iter(self.response_cache))
                del self.response_cache[oldest_key]

    def batch_process_commands(self, commands: List[str]) -> List[Dict]:
        """
        Batch process multiple commands for efficiency
        """
        # In a real implementation, this would send multiple commands
        # to the LLM in a single request
        results = []

        for command in commands:
            # Check cache first
            cache_key = hashlib.md5(command.encode()).hexdigest()
            if cache_key in self.response_cache:
                results.append(self.response_cache[cache_key])
            else:
                # Process normally
                result = self.process_single_command(command)
                results.append(result)

                # Cache if appropriate
                self.intelligent_caching(command, result)

        return results

    def adaptive_prompting(self, command_complexity: str) -> str:
        """
        Use different prompting strategies based on command complexity
        """
        if command_complexity == "simple":
            return "simple_direct"
        elif command_complexity == "moderate":
            return "few_shot"
        else:  # complex
            return "chain_of_thought"
```

### Real-time Performance Considerations

Optimizing for real-time robotic applications:

```python
# Example: Real-time performance optimization
class RealTimeCognitivePlanner:
    def __init__(self, llm_bridge: LLMBridge):
        self.llm_bridge = llm_bridge
        self.max_response_time = 2.0  # Maximum time to wait for LLM response
        self.fallback_planner = SimpleFallbackPlanner()  # Fallback for time-critical tasks

    def process_with_timeout(self, prompt: str, timeout: float = 2.0) -> Optional[Dict]:
        """
        Process command with timeout to ensure real-time performance
        """
        import concurrent.futures

        # Submit LLM call to thread pool with timeout
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(self.llm_bridge.call_llm, prompt)

            try:
                result = future.result(timeout=timeout)
                return self.llm_bridge.validate_response(result)
            except concurrent.futures.TimeoutError:
                self.get_logger().warn("LLM call timed out, using fallback")
                return self.fallback_planner.process(prompt)

    def predictive_planning(self, context: Dict) -> List[Dict]:
        """
        Pre-generate likely action sequences based on context
        """
        # In high-level applications, predict likely next commands
        # and pre-generate action sequences
        predicted_commands = self.predict_next_commands(context)
        precomputed_actions = {}

        for cmd in predicted_commands:
            action_seq = self.process_with_timeout(
                self.prompter.create_planning_prompt(cmd, context),
                timeout=1.0  # Shorter timeout for predictions
            )
            precomputed_actions[cmd] = action_seq

        return precomputed_actions
```

## Practical Examples

### Example 1: Simple Navigation Planning

```python
# Example: Simple navigation command planning
def example_simple_navigation():
    """
    Example of planning a simple navigation command
    """
    llm_bridge = LLMBridge()
    prompter = CognitivePlanningPrompter()
    safety_validator = SafetyValidator()

    # User command
    command = "Please go to the kitchen"

    # Create planning prompt
    prompt = prompter.create_planning_prompt(
        command,
        robot_state={"x": 0.0, "y": 0.0, "theta": 0.0, "location": "living_room"},
        environment_info={"kitchen_location": {"x": 5.0, "y": 3.0}}
    )

    # Get LLM response
    response = llm_bridge.call_llm(prompt)
    action_sequence = llm_bridge.validate_response(response)

    print(f"Command: {command}")
    print(f"Generated actions: {json.dumps(action_sequence, indent=2)}")

    # Validate for safety
    validation = safety_validator.validate_action_sequence(
        action_sequence.get('actions', []), command
    )
    print(f"Safety validation: {validation}")

    return action_sequence

# Run the example
navigation_plan = example_simple_navigation()
```

### Example 2: Complex Multi-Step Task

Implementing a complex task that requires multiple cognitive planning steps.

## Troubleshooting and Best Practices

### Common Issues and Solutions

1. **LLM Inconsistency**
   - Use structured output format (JSON) to ensure consistent parsing
   - Implement validation layers to check for expected output structure
   - Use temperature < 0.5 for more deterministic outputs

2. **Performance Bottlenecks**
   - Implement caching for frequently used commands
   - Use smaller models for simple tasks, larger models for complex reasoning
   - Consider local models for privacy and performance

3. **Safety Concerns**
   - Always implement multiple layers of safety validation
   - Use human-in-the-loop for critical operations
   - Maintain fallback mechanisms for safety-critical scenarios

### Monitoring and Evaluation

```python
# Example: Monitoring and evaluation for cognitive planning
class CognitivePlanningMonitor:
    def __init__(self):
        self.planning_times = []
        self.success_rates = []
        self.safety_violations = []
        self.user_satisfaction = []

    def log_planning_event(self, command: str, action_sequence: List[Dict],
                          planning_time: float, success: bool):
        """
        Log a cognitive planning event for monitoring
        """
        self.planning_times.append(planning_time)

        if success:
            self.success_rates.append(1)
        else:
            self.success_rates.append(0)

    def evaluate_planning_quality(self, original_command: str,
                                generated_actions: List[Dict]) -> Dict:
        """
        Evaluate the quality of generated action sequences
        """
        metrics = {
            'completeness': 0.0,  # How well the plan addresses the command
            'efficiency': 0.0,    # How efficient the action sequence is
            'safety': 0.0,        # Safety rating of the plan
            'naturalness': 0.0    # How natural the plan seems
        }

        # Calculate metrics based on various factors
        # This is a simplified example
        if generated_actions:
            metrics['completeness'] = min(1.0, len(generated_actions) / 10)  # Arbitrary scale
            metrics['efficiency'] = 1.0 / len(generated_actions)  # Simpler is more efficient
            metrics['naturalness'] = 0.8  # Default assumption

        return metrics

    def get_performance_report(self) -> str:
        """
        Generate a performance report
        """
        if not self.planning_times:
            return "No data collected yet"

        avg_time = sum(self.planning_times) / len(self.planning_times)
        success_rate = sum(self.success_rates) / len(self.success_rates) if self.success_rates else 0

        report = f"""
        Cognitive Planning Performance Report:
        - Average planning time: {avg_time:.2f}s
        - Success rate: {success_rate:.2%}
        - Total plans generated: {len(self.planning_times)}
        - Safety violations: {len(self.safety_violations)}
        """

        return report
```

## Exercises

1. **Prompt Engineering**: Design and test different prompt structures to optimize LLM performance for cognitive planning
2. **Safety Validation**: Implement additional safety rules and validation mechanisms for robotic action sequences
3. **Context Management**: Create a more sophisticated context management system that handles complex multi-turn interactions
4. **Performance Optimization**: Implement caching and optimization strategies to improve real-time performance

## Summary

This chapter has covered cognitive planning using LLMs to convert natural language commands into ROS 2 action sequences. You've learned about prompt engineering, safety validation, context management, and integration with the ROS 2 ecosystem. The foundation laid here will be essential for the complete VLA integration covered in the next chapter.

## Next Steps

In the next chapter, we'll explore the complete Vision-Language-Action integration, combining voice processing, cognitive planning, and robotic execution into a unified system for autonomous humanoid task execution.