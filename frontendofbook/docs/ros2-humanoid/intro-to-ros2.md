---
sidebar_position: 1
---

# Introduction to ROS 2 for Physical AI

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and environments.

For humanoid robotics, ROS 2 provides the essential middleware that allows different parts of the robot to communicate with each other effectively. Whether it's the perception system talking to the planning system, or the control system receiving commands from the user interface, ROS 2 handles the communication infrastructure.

## Why ROS 2 Matters for Humanoid Robotics

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

- **Complexity**: Humanoid robots have many degrees of freedom and subsystems that need to work together
- **Distributed Nature**: Different parts of the robot (arms, legs, head, sensors) often run on separate computers
- **Real-time Requirements**: Many humanoid robot functions require real-time processing and response
- **Modularity**: Different teams may work on different aspects of the robot simultaneously

ROS 2 addresses these challenges by providing:

1. **Distributed Computing**: Nodes can run on different machines and communicate seamlessly
2. **Real-time Support**: With proper configuration, ROS 2 can meet real-time requirements
3. **Language Independence**: Components can be written in different programming languages
4. **Package Management**: Reusable components can be easily shared and integrated

## DDS Concepts

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. DDS is a specification that defines a standard for data-centric connectivity.

### Data-Centric vs Service-Centric

Traditional communication systems are often service-centric, meaning they focus on connecting services or functions. DDS, however, is data-centric, meaning it focuses on the data itself. This approach has several advantages:

- **Decoupling**: Publishers and subscribers don't need to know about each other
- **Scalability**: Multiple subscribers can receive the same data without publishers knowing
- **Flexibility**: New components can be added without modifying existing ones

### DDS Quality of Service (QoS)

DDS provides Quality of Service policies that allow you to specify how data should be delivered:

- **Reliability**: Whether messages must be delivered (RELIABLE) or can be lost (BEST_EFFORT)
- **Durability**: Whether late-joining subscribers get previous messages (TRANSIENT_LOCAL) or not (VOLATILE)
- **History**: How many messages to keep for late joiners
- **Deadline**: How often data is expected to arrive
- **Lifespan**: How long published data is valid

These QoS policies are crucial for humanoid robots where some data (like sensor readings) might need to be reliable while other data (like debug information) can be best-effort.

## ROS 2 Architecture

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In a humanoid robot, you might have nodes for:

- Sensor processing
- Motion planning
- Control systems
- Perception
- User interfaces

### Packages

Packages are the basic building and distribution units in ROS 2. They contain:

- Source code
- Configuration files
- Launch files
- Documentation
- Dependencies

### Topics and Messages

Topics are named buses over which nodes exchange messages. Messages are the data structures that are passed between nodes. For example, a camera node might publish images on a `/camera/image_raw` topic using a `sensor_msgs/Image` message type.

### Services and Actions

Services provide request-response communication, useful for operations that have a clear beginning and end. Actions are for long-running tasks that might be preempted, making them ideal for robot navigation and manipulation tasks.

## Getting Started with ROS 2

To start working with ROS 2 for humanoid robotics, you'll need to:

1. Install ROS 2 (Humble Hawksbill is the current LTS version)
2. Set up your development environment
3. Learn the basic tools (ros2 CLI, rqt, rviz)
4. Understand the build system (colcon)
5. Practice with simple examples before moving to humanoid-specific packages

## Summary

ROS 2 provides the essential communication infrastructure for humanoid robots, enabling different subsystems to work together seamlessly. Its use of DDS as the underlying middleware provides flexibility, scalability, and real-time capabilities that are essential for humanoid robotics applications. Understanding these foundational concepts is crucial before diving into the communication model and robot structure topics.