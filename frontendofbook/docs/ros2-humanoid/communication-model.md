---
sidebar_position: 2
---

# ROS 2 Communication Model

## Understanding Nodes

In ROS 2, a node is the fundamental unit of execution. It's a process that performs computation and communicates with other nodes through the ROS graph. For humanoid robots, nodes represent different functional components:

- **Sensor nodes**: Handle data from cameras, IMUs, joint encoders, etc.
- **Controller nodes**: Manage robot motion and control algorithms
- **Perception nodes**: Process sensor data to understand the environment
- **Planning nodes**: Generate motion plans and trajectories
- **Behavior nodes**: Coordinate high-level robot behaviors

### Creating a Node with rclpy

The Python client library for ROS 2 (rclpy) provides the tools to create nodes. Here's a basic example of a node structure:

```python
import rclpy
from rclpy.node import Node

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        # Initialize publishers, subscribers, services, etc.
        self.get_logger().info('Humanoid Controller Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Publishers/Subscribers

Topics enable asynchronous, many-to-many communication in ROS 2. Data is published to a topic and any number of nodes can subscribe to receive that data.

### Publishers

A publisher sends messages to a topic. Here's an example of publishing joint commands for a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_commands)  # 10Hz
        self.step = 0

    def publish_joint_commands(self):
        msg = JointState()
        msg.name = ['left_hip', 'left_knee', 'left_ankle', 'right_hip', 'right_knee', 'right_ankle']

        # Generate simple oscillating joint commands
        positions = []
        for i in range(6):
            pos = math.sin(self.step * 0.1 + i) * 0.5
            positions.append(pos)

        msg.position = positions
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscribers

A subscriber receives messages from a topic. Here's an example of subscribing to sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for processed data
        self.error_publisher = self.create_publisher(Float64, 'position_error', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

        # Process the sensor data (example: calculate position error)
        error_msg = Float64()
        error_msg.data = 0.0  # Placeholder for actual error calculation
        self.error_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide synchronous, request-response communication. This is useful for operations that have a clear beginning and end, such as calibration or configuration changes.

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')
        self.srv = self.create_service(
            SetBool,
            'calibrate_robot',
            self.calibrate_robot_callback)
        self.calibrated = False

    def calibrate_robot_callback(self, request, response):
        if request.data:  # If request is to calibrate
            self.get_logger().info('Starting robot calibration...')
            # Perform calibration (simplified)
            self.calibrated = True
            response.success = True
            response.message = 'Robot calibration completed successfully'
        else:
            response.success = False
            response.message = 'Calibration request rejected'

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.cli = self.create_client(SetBool, 'calibrate_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, calibrate=True):
        self.req.data = calibrate
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = CalibrationClient()
    response = client.send_request(True)

    if response:
        print(f'Result: {response.success}, Message: {response.message}')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions are designed for long-running tasks that might be preempted. They're ideal for humanoid robot tasks like navigation, manipulation, or complex motion sequences.

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Basic rclpy-based Agent Controller Flow

A humanoid robot controller typically follows this flow pattern:

1. **Initialization**: Set up ROS 2 node, publishers, subscribers, timers
2. **Sensor Data Acquisition**: Subscribe to sensor topics or request data
3. **State Estimation**: Process sensor data to understand robot state
4. **Planning/Decision Making**: Determine desired actions based on state and goals
5. **Control Output**: Publish commands to actuators
6. **Loop**: Repeat steps 2-5 at the required frequency

Here's a complete example of a simple humanoid balance controller:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers for sensor data
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Publisher for joint commands
        self.command_pub = self.create_publisher(
            Float64MultiArray, 'joint_commands', 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz

        # Internal state
        self.current_imu = None
        self.current_joints = None
        self.target_positions = [0.0] * 12  # Example: 12 joints

    def imu_callback(self, msg):
        self.current_imu = msg

    def joint_callback(self, msg):
        self.current_joints = msg

    def control_loop(self):
        if self.current_imu is None or self.current_joints is None:
            return  # Wait for sensor data

        # Simple balance control (simplified example)
        roll = self.current_imu.orientation.x
        pitch = self.current_imu.orientation.y

        # Adjust target based on balance error
        self.target_positions[0] += pitch * 0.1  # Adjust hip joints based on pitch
        self.target_positions[1] -= roll * 0.1   # Adjust based on roll

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.target_positions
        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Considerations for Humanoid Robotics

Different types of data in humanoid robots require different QoS settings:

- **Sensor data**: Usually RELIABLE with small history for real-time processing
- **Control commands**: RELIABLE with small history to ensure commands are received
- **Debug information**: BEST_EFFORT with small history to reduce overhead
- **Configuration data**: RELIABLE with TRANSIENT_LOCAL durability so new nodes get current config

## Summary

The ROS 2 communication model provides flexible and robust ways to connect different parts of a humanoid robot system. Understanding nodes, topics, services, and actions is crucial for building effective robot controllers. The rclpy library provides Python access to all these communication patterns, making it accessible for AI developers to implement complex humanoid robot behaviors.