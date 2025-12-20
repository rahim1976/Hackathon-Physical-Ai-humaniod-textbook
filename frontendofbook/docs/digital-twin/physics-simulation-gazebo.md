---
sidebar_position: 1
title: Physics Simulation with Gazebo
---

# Physics Simulation with Gazebo

## Introduction to Gazebo Physics Simulation for Humanoid Robots

This chapter covers the fundamentals of physics simulation using Gazebo for humanoid robotics applications. Gazebo is a powerful robotics simulation environment that provides accurate physics simulation, realistic rendering, and convenient programmatic interfaces.

### Why Gazebo Matters for Humanoid Robotics

Gazebo is essential for humanoid robotics development because it provides:

1. **Realistic Physics Simulation**: Accurate modeling of forces, collisions, and dynamics that humanoid robots experience
2. **Safe Testing Environment**: Test complex behaviors without risk of damaging expensive hardware
3. **Rapid Prototyping**: Quickly iterate on robot designs and control algorithms
4. **Cost-Effective Development**: Reduce the need for physical prototypes during early development phases
5. **Reproducible Experiments**: Consistent testing conditions that can be replicated across teams

The physics simulation capabilities in Gazebo are particularly important for humanoid robots, which have complex multi-link structures with many degrees of freedom that must interact with the environment in sophisticated ways.

### Learning Objectives
- Understand the core concepts of Gazebo physics simulation
- Learn about different physics engines available in Gazebo (ODE, Bullet, Simbody)
- Master URDF integration with Gazebo for humanoid robot models
- Configure joint constraints and motor controls
- Implement collision detection and response mechanisms
- Apply best practices for physics parameter tuning for humanoid robots

### Prerequisites
- Basic understanding of robotics concepts (kinematics, dynamics)
- Familiarity with ROS/ROS2 (recommended but not required)
- Basic programming skills in Python or C++

## Gazebo's Physics Engines

Gazebo supports multiple physics engines that provide different capabilities for simulation, each with specific strengths for humanoid robotics applications:

### ODE (Open Dynamics Engine)
- Default physics engine in many Gazebo versions
- Good performance for most robotics applications
- Supports rigid body dynamics and basic collision detection
- Well-tested and stable for humanoid robot simulations
- Optimal for real-time applications with moderate accuracy requirements
- Best suited for: Walking algorithms, basic manipulation, environment interaction

### Bullet Physics
- High-performance physics engine
- Excellent for complex collision detection
- Good support for constraints and joints
- Faster than ODE for complex scenarios
- Better handling of complex contact situations
- Best suited for: High-fidelity contact simulation, complex environments, multi-robot scenarios

### Simbody
- Multibody dynamics engine
- High-fidelity simulation capabilities
- Particularly good for biomechanical simulations
- More accurate for complex articulated systems
- Better energy conservation properties
- Best suited for: Biomechanically-inspired robots, high-precision applications, research scenarios

### Choosing the Right Physics Engine for Humanoid Robots

The choice of physics engine depends on your specific humanoid robot requirements:

- **For real-time control**: ODE or Bullet provide good balance of performance and accuracy
- **For high-fidelity simulation**: Simbody offers superior accuracy for complex dynamics
- **For complex contacts**: Bullet handles multiple contact points better than ODE
- **For stability**: ODE has proven stability in long-running simulations

#### Configuration Example for Physics Engine Selection

```xml
<!-- In your world file -->
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Choose physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Alternative: Using Bullet -->
    <!--
    <physics type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    -->
  </world>
</sdf>
```

## URDF Integration with Gazebo

The Unified Robot Description Format (URDF) is the standard format for describing robots in ROS. Integrating URDF with Gazebo requires specific tags and configurations that define how the robot behaves in the simulation environment.

### Complete Humanoid Robot URDF Example

Here's a more comprehensive example of a humanoid robot URDF that includes proper physics properties for Gazebo simulation:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Pelvis Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Left Arm Links -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints connecting the links -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <joint name="head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 1.5708"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

  <!-- Gazebo-specific tags -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Gazebo plugins for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/simple_humanoid</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>
</robot>
```

### Gazebo-Specific URDF Tags

The `<gazebo>` tags in URDF allow you to specify simulation-specific properties:

- **`<material>`**: Defines visual appearance in Gazebo
- **`<damping>`**: Controls motion damping for more realistic behavior
- **`<mu1>`, `<mu2>`**: Friction coefficients for contact simulation
- **`<kp>`, `<kd>`**: Contact stiffness and damping parameters
- **`<maxVel>`**, `<minDepth>`**: Contact constraint parameters

### Physics Parameters for Humanoid Robots

Proper physics parameters are crucial for stable humanoid simulation:

- **Mass values**: Should reflect realistic robot weights
- **Inertia tensors**: Should be physically plausible for the link shapes
- **Damping values**: Help stabilize joint movements
- **Friction coefficients**: Affect ground contact and manipulation
- **Joint limits**: Prevent damage and ensure realistic motion ranges

## Joint Constraints and Motor Controls

### Joint Types in Gazebo for Humanoid Robots

Gazebo supports several joint types that are essential for humanoid robot simulation. Each joint type serves specific purposes in mimicking human-like movement:

- **Fixed joints**: Rigid connections between links (e.g., attaching sensors to the robot body)
- **Revolute joints**: Rotational joints with limited range (e.g., elbow, knee joints)
- **Continuous joints**: Rotational joints without limits (e.g., shoulder, hip joints with full rotation)
- **Prismatic joints**: Linear sliding joints (rarely used in humanoid robots)
- **Floating joints**: 6-DOF unconstrained joints (used for floating base simulation)
- **Planar joints**: Motion constrained to a plane (rarely used in humanoid robots)
- **Spherical joints**: Ball-and-socket joints with 3-DOF rotation (useful for shoulder/hip joints)

### Joint Parameters for Humanoid Robots

Proper joint configuration is critical for realistic humanoid movement:

- **`<limit>`**: Defines joint limits, effort, and velocity constraints
- **`<dynamics>`**: Sets damping and friction parameters
- **`<safety_controller>`**: Provides safety limits to prevent damage
- **`<calibration>`**: Defines joint zero position
- **`<mimic>`**: Creates coupled joint relationships

### Advanced Motor Control Implementation

For sophisticated humanoid control, you'll need to implement proper ROS control interfaces:

```xml
<!-- Joint definitions with proper control parameters -->
<joint name="left_hip_yaw_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_thigh"/>
  <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.0" upper="1.0" effort="200" velocity="5"/>
  <dynamics damping="1.0" friction="0.1"/>
  <safety_controller k_position="20" k_velocity="400" soft_lower_limit="-0.9" soft_upper_limit="0.9"/>
</joint>

<joint name="left_hip_roll_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.5" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.5" upper="1.5" effort="200" velocity="5"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>

<!-- Transmission for ROS control -->
<transmission name="left_hip_yaw_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_yaw_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_yaw_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!-- Gazebo plugin for ROS control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/simple_humanoid</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```

### Joint Control Strategies for Humanoid Robots

Different control strategies work best for different aspects of humanoid locomotion:

1. **Position Control**: Good for precise pose following, commonly used for manipulation
2. **Velocity Control**: Useful for smooth motion, good for walking patterns
3. **Effort Control**: Provides direct force control, essential for compliant behaviors
4. **Impedance Control**: Allows for variable stiffness, important for safe human interaction

### Example Controller Configuration

```yaml
# Controllers configuration file (controllers.yaml)
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

# Position controllers for each joint
left_hip_yaw_position_controller:
  type: position_controllers/JointPositionController
  joint: left_hip_yaw_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}

left_hip_roll_position_controller:
  type: position_controllers/JointPositionController
  joint: left_hip_roll_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}

# Effort controllers for compliance
left_ankle_effort_controller:
  type: effort_controllers/JointEffortController
  joint: left_ankle_joint
  pid: {p: 10.0, i: 0.1, d: 0.5}
```

## Collision Detection and Response

### Collision Properties for Humanoid Robots

Gazebo provides sophisticated collision detection and response mechanisms that are crucial for humanoid robot simulation, especially for tasks like walking, balance, and manipulation:

- **Contact sensors**: Detect when robot parts touch the environment or self-collide
- **Collision checking**: Determine if robot links intersect with environment objects
- **Response physics**: Calculate contact forces and reactions for realistic interaction
- **Ground contact**: Essential for stable walking and balance simulation
- **Self-collision avoidance**: Prevents robot from intersecting with itself

### Surface Properties for Realistic Contact

For humanoid robots, proper surface properties are essential for realistic ground contact and manipulation:

```xml
<gazebo reference="left_foot_link">
  <collision name="left_foot_collision">
    <surface>
      <contact>
        <ode>
          <!-- Constraint Force Mixing (CFM) - affects constraint compliance -->
          <soft_cfm>0.001</soft_cfm>
          <!-- Error Reduction Parameter (ERP) - affects constraint stiffness -->
          <soft_erp>0.9</soft_erp>
          <kp>1e8</kp>  <!-- Spring stiffness -->
          <kd>1e4</kd>  <!-- Damping coefficient -->
          <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
          <min_depth>0.001</min_depth>  <!-- Penetration depth before contact force applied -->
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Primary friction coefficient -->
          <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
          <fdir1>0 0 0</fdir1>  <!-- Friction direction -->
          <slip1>0.0</slip1>  <!-- Primary slip coefficient -->
          <slip2>0.0</slip2>  <!-- Secondary slip coefficient -->
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.01</restitution_coefficient>  <!-- Bounciness -->
        <threshold>10.0</threshold>  <!-- Velocity threshold for bounce -->
      </bounce>
    </surface>
  </collision>
</gazebo>
```

### Contact Sensors for Feedback

Contact sensors provide crucial feedback for humanoid robot control:

```xml
<gazebo reference="left_foot_link">
  <sensor name="left_foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <plugin name="left_foot_contact_plugin" filename="libgazebo_ros_bumper.so">
      <alwaysOn>true</alwaysOn>
      <frameName>left_foot_link</frameName>
      <topicName>left_foot_bumper</topicName>
      <bumperTopicName>left_foot_contact_states</bumperTopicName>
    </plugin>
  </sensor>
</gazebo>
```

### Physics Parameters for Humanoid Balance

Proper physics parameters are essential for stable humanoid simulation:

- **CFM (Constraint Force Mixing)**: Lower values = stiffer constraints, but potentially unstable
- **ERP (Error Reduction Parameter)**: Higher values = faster error correction, but potentially oscillatory
- **kp/kd**: Spring stiffness and damping for contact response
- **Friction coefficients**: Affect grip and stability during walking
- **Contact depth**: Affects how much penetration is allowed before forces are applied

### Tuning for Humanoid Applications

For humanoid robots, consider these tuning guidelines:

- **Walking stability**: Use higher ERP (0.8-0.95) and moderate CFM (0.0001-0.001)
- **Manipulation**: Use higher stiffness (kp = 1e8) for precise contact
- **Balance**: Adjust friction coefficients (0.7-1.0) for realistic ground contact
- **Performance**: Lower update rates (50-100Hz) for faster simulation, higher rates (500-1000Hz) for accuracy

### Example: Complete Foot Configuration

```xml
<!-- Complete foot configuration optimized for humanoid walking -->
<link name="left_foot">
  <visual>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
  </inertial>
</link>

<gazebo reference="left_foot">
  <!-- Visual properties -->
  <material>Gazebo/Black</material>

  <!-- Collision properties optimized for walking -->
  <collision name="left_foot_collision">
    <surface>
      <contact>
        <ode>
          <soft_cfm>0.0001</soft_cfm>
          <soft_erp>0.9</soft_erp>
          <kp>1e8</kp>
          <kd>1e4</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.0005</min_depth>
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>0.9</mu>
          <mu2>0.9</mu2>
        </ode>
      </friction>
    </surface>
  </collision>

  <!-- Contact sensor for ground contact detection -->
  <sensor name="left_foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
  </sensor>
</gazebo>
```

## Exercises

### Exercise 1: Basic Humanoid Robot Model
Create a simple humanoid robot model with at least 5 links and 4 joints in Gazebo:
1. Design a robot with pelvis, torso, head, and two arms
2. Implement proper URDF with visual, collision, and inertial properties
3. Add Gazebo-specific tags for visualization and physics
4. Test the model in Gazebo to ensure it's stable and properly configured

### Exercise 2: Physics Parameter Tuning
Adjust physics parameters to achieve stable behavior:
1. Experiment with different ERP and CFM values to find stable configurations
2. Tune joint damping and friction for realistic movement
3. Test the robot's response to external forces
4. Document the optimal parameters for your specific robot model

### Exercise 3: Collision Detection and Response
Implement comprehensive collision detection:
1. Add contact sensors to feet and hands for environmental interaction
2. Configure surface properties for realistic ground contact
3. Test collision detection between robot and environment objects
4. Implement a simple balance controller using contact feedback

### Exercise 4: Advanced Joint Control
Implement sophisticated joint control:
1. Set up ROS control interfaces for your robot
2. Create position, velocity, and effort controllers
3. Test different control strategies (position, velocity, effort)
4. Implement a simple walking pattern using joint trajectories

## Summary

This chapter introduced the fundamentals of physics simulation with Gazebo for humanoid robots. We covered physics engines, URDF integration, joint constraints, and collision detection. Proper physics simulation is crucial for developing and testing humanoid robot behaviors in a safe, controlled environment. The next chapter will explore digital twins and HRI in Unity.