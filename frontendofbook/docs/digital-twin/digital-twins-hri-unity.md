---
sidebar_position: 2
title: Digital Twins & HRI in Unity
---

# Digital Twins & HRI in Unity

## Introduction to Unity for Robotics Digital Twins

This chapter explores the use of Unity for creating high-fidelity digital twins and human-robot interaction (HRI) environments. Unity's powerful rendering capabilities and real-time performance make it an ideal platform for creating immersive visualization and interaction experiences for humanoid robots.

### The Role of Digital Twins in Robotics

Digital twins serve as virtual replicas of physical systems, enabling:

1. **Real-time Monitoring**: Continuous visualization of robot state and environment
2. **Predictive Analysis**: Anticipating robot behavior and potential issues
3. **Remote Operation**: Controlling robots from a safe distance through virtual interfaces
4. **Training and Simulation**: Providing safe environments for operator training
5. **System Optimization**: Testing improvements and modifications virtually before implementation

For humanoid robots, digital twins are particularly valuable as they allow for:
- Complex multi-joint visualization and monitoring
- Realistic environment interaction simulation
- Safe testing of complex behaviors
- Enhanced human-robot collaboration scenarios

### Why Unity for Robotics Digital Twins

Unity offers several advantages for robotics digital twin applications:

- **High-Fidelity Rendering**: Professional-grade graphics for realistic visualization
- **Real-time Performance**: Smooth, interactive experiences for live robot monitoring
- **Cross-Platform Deployment**: Run on various devices from desktops to mobile AR
- **Asset Ecosystem**: Extensive library of 3D models, materials, and tools
- **VR/AR Support**: Native support for immersive interaction experiences
- **Scripting Flexibility**: C# scripting for custom robot interfaces and behaviors

### Learning Objectives
- Understand Unity's rendering pipeline for digital twin applications
- Learn about real-time synchronization between Gazebo and Unity
- Master human-robot interaction design principles in Unity
- Explore VR/AR integration possibilities for enhanced HRI
- Optimize Unity scenes for real-time visualization
- Implement effective HRI interfaces for humanoid robot control

### Prerequisites
- Basic understanding of 3D graphics concepts
- Familiarity with Unity development environment
- Knowledge of humanoid robot simulation from Chapter 1

## Unity's Rendering Pipeline for Digital Twins

Unity's rendering pipeline provides the foundation for creating realistic digital twin environments. Choosing the right pipeline depends on your performance requirements and visual quality needs:

### Universal Render Pipeline (URP)
- **Performance**: Lightweight and efficient for real-time applications
- **Compatibility**: Good performance on a wide range of hardware (mobile, PC, console)
- **Features**: 2D and 3D rendering, lighting, shadows, post-processing
- **Best for**: Robotics applications requiring real-time performance with good visual quality
- **Hardware requirements**: Moderate, suitable for edge computing scenarios

### High Definition Render Pipeline (HDRP)
- **Visual Quality**: Advanced rendering features for high-fidelity visuals
- **Physically-based Rendering**: Accurate materials and lighting simulation
- **Features**: Volumetric lighting, advanced shadows, ray tracing
- **Best for**: High-end visualization, detailed robot inspection, photorealistic environments
- **Hardware requirements**: High-end GPUs, typically not suitable for real-time robot control

### Built-in Render Pipeline
- **Compatibility**: Legacy pipeline with wide compatibility across Unity versions
- **Simplicity**: Straightforward lighting and shading models
- **Performance**: Good for simpler visualization needs
- **Best for**: Basic digital twin applications, quick prototyping

### Recommended Pipeline for Robotics Digital Twins

For most humanoid robotics digital twin applications, **URP is recommended** because:
- It provides excellent performance for real-time robot visualization
- It has reasonable hardware requirements for deployment
- It offers sufficient visual quality for operator understanding
- It supports both 3D robot visualization and 2D interface elements

#### URP Setup for Robotics Visualization

```csharp
// Example: Setting up URP for robotics visualization
using UnityEngine;
using UnityEngine.Rendering.Universal;

public class RobotVisualizationSetup : MonoBehaviour
{
    [Header("Robot Visualization Settings")]
    public float robotScale = 1.0f;
    public Color robotColor = Color.gray;
    public bool showJointAxes = true;
    public bool showTrajectory = true;

    private void Start()
    {
        // Configure URP-specific settings for robotics
        ConfigureRobotRendering();
    }

    private void ConfigureRobotRendering()
    {
        // Set up robot-specific rendering properties
        var universalRenderer = (UniversalRenderer)RenderPipelineManager.currentPipeline.rendererList[0];

        // Configure for robot visualization
        RenderSettings.ambientLight = new Color(0.4f, 0.4f, 0.4f, 1);
        RenderSettings.fog = true;
        RenderSettings.fogColor = Color.gray;
        RenderSettings.fogDensity = 0.01f;
    }

    // Update robot visualization based on real-time data
    public void UpdateRobotVisualization(RobotState robotState)
    {
        // Update joint positions based on robot state
        foreach (var joint in robotState.joints)
        {
            Transform jointTransform = FindJointTransform(joint.name);
            if (jointTransform != null)
            {
                jointTransform.localRotation = Quaternion.Euler(0, 0, joint.position * Mathf.Rad2Deg);
            }
        }
    }

    private Transform FindJointTransform(string jointName)
    {
        // Implementation to find the corresponding joint transform
        // This would map robot joint names to Unity transforms
        return transform.Find(jointName);
    }
}
```

## Real-time Synchronization between Gazebo and Unity

### ROS# Integration for Robotics Communication

Unity can connect to ROS/ROS2 systems using the ROS# package, which provides a bridge between Unity and the Robot Operating System:

```csharp
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Sensor_msgs;
using RosSharp.Messages.Geometry_msgs;
using RosSharp.Messages.Std_msgs;
using System.Collections.Generic;

public class RobotController : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    public string rosBridgeServerUrl = "ws://localhost:9090";
    public string robotNamespace = "/humanoid_robot";

    [Header("Robot State Topics")]
    public string jointStatesTopic = "/joint_states";
    public string tfTopic = "/tf";

    private RosSocket rosSocket;
    private Dictionary<string, Transform> jointMap = new Dictionary<string, Transform>();

    // Interpolation for smooth visualization
    private Dictionary<string, float> targetJointPositions = new Dictionary<string, float>();
    private Dictionary<string, float> currentJointPositions = new Dictionary<string, float>();
    private float interpolationSpeed = 10.0f;

    void Start()
    {
        // Initialize joint mapping
        InitializeJointMap();

        // Connect to ROS bridge
        ConnectToRosBridge();
    }

    private void InitializeJointMap()
    {
        // Map robot joint names to Unity transforms
        // This should match the joint names in your URDF
        jointMap["left_hip_yaw_joint"] = transform.Find("LeftHipYaw");
        jointMap["left_hip_roll_joint"] = transform.Find("LeftHipRoll");
        jointMap["left_hip_pitch_joint"] = transform.Find("LeftHipPitch");
        jointMap["left_knee_joint"] = transform.Find("LeftKnee");
        jointMap["left_ankle_pitch_joint"] = transform.Find("LeftAnklePitch");
        jointMap["left_ankle_roll_joint"] = transform.Find("LeftAnkleRoll");

        // Initialize position dictionaries
        foreach (var jointName in jointMap.Keys)
        {
            if (jointMap[jointName] != null)
            {
                currentJointPositions[jointName] = jointMap[jointName].localEulerAngles.z;
                targetJointPositions[jointName] = jointMap[jointName].localEulerAngles.z;
            }
        }
    }

    private void ConnectToRosBridge()
    {
        try
        {
            rosSocket = new RosSocket(new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl));

            // Subscribe to joint states
            rosSocket.Subscribe<JointState>(robotNamespace + jointStatesTopic, JointStateHandler);

            // Subscribe to other relevant topics
            rosSocket.Subscribe<TFMessage>(robotNamespace + tfTopic, TransformHandler);

            Debug.Log("Connected to ROS bridge: " + rosBridgeServerUrl);
        }
        catch (System.Exception e)
        {
            Debug.LogError("Failed to connect to ROS bridge: " + e.Message);
        }
    }

    void JointStateHandler(JointState jointState)
    {
        // Update target positions for interpolation
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = (float)jointState.position[i];

            if (targetJointPositions.ContainsKey(jointName))
            {
                targetJointPositions[jointName] = jointPosition * Mathf.Rad2Deg;
            }
        }
    }

    void TransformHandler(TFMessage tfMessage)
    {
        // Handle transform updates for robot pose
        foreach (var transform in tfMessage.transforms)
        {
            // Process transform data for visualization
            ProcessTransform(transform);
        }
    }

    private void ProcessTransform(TransformStamped transform)
    {
        // Update robot position/pose in Unity based on TF data
        string frameId = transform.child_frame_id;
        var translation = transform.transform.translation;
        var rotation = transform.transform.rotation;

        // Apply transform to corresponding Unity object
        Transform unityTransform = FindUnityTransform(frameId);
        if (unityTransform != null)
        {
            unityTransform.position = new Vector3((float)translation.x, (float)translation.y, (float)translation.z);
            unityTransform.rotation = new Quaternion((float)rotation.x, (float)rotation.y, (float)rotation.z, (float)rotation.w);
        }
    }

    private Transform FindUnityTransform(string frameId)
    {
        // Convert ROS frame ID to Unity transform path
        string unityPath = frameId.Replace("/", "");
        if (string.IsNullOrEmpty(unityPath)) unityPath = "base_link";

        return transform.Find(unityPath);
    }

    void Update()
    {
        // Interpolate joint positions for smooth visualization
        foreach (var jointName in jointMap.Keys)
        {
            if (jointMap[jointName] != null && targetJointPositions.ContainsKey(jointName))
            {
                currentJointPositions[jointName] = Mathf.Lerp(
                    currentJointPositions[jointName],
                    targetJointPositions[jointName],
                    Time.deltaTime * interpolationSpeed
                );

                jointMap[jointName].localRotation = Quaternion.Euler(0, 0, currentJointPositions[jointName]);
            }
        }
    }

    private void OnDestroy()
    {
        // Clean up ROS connection
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}
```

### Advanced Synchronization Techniques

#### Data Buffering for Smooth Visualization

```csharp
using System.Collections.Generic;
using UnityEngine;

public class RobotStateBuffer
{
    private Queue<RobotState> stateBuffer = new Queue<RobotState>();
    private int maxBufferSize = 10;

    public void AddState(RobotState state)
    {
        if (stateBuffer.Count >= maxBufferSize)
        {
            stateBuffer.Dequeue(); // Remove oldest state
        }
        stateBuffer.Enqueue(state);
    }

    public RobotState GetInterpolatedState(float deltaTime)
    {
        if (stateBuffer.Count < 2) return null;

        RobotState latest = stateBuffer.Peek();
        RobotState previous = stateBuffer.ToArray()[1];

        // Interpolate between states based on deltaTime
        RobotState interpolated = new RobotState();
        // Implementation of state interpolation
        return interpolated;
    }
}

public class RobotState
{
    public Dictionary<string, float> jointPositions = new Dictionary<string, float>();
    public Vector3 position;
    public Quaternion rotation;
    public float timestamp;
}
```

### Custom Synchronization Protocol

For direct Gazebo-Unity communication without ROS:

1. **Data Format**: Use JSON or Protocol Buffers for efficient data transmission
2. **Update Frequency**: Balance between real-time performance and network load
3. **Interpolation**: Smooth transitions between states for visual quality
4. **Error Handling**: Robust connection management for production systems
5. **Data Compression**: Optimize bandwidth usage for remote applications

#### WebSocket-based Direct Communication

```csharp
using System;
using System.Collections;
using UnityEngine;
using Newtonsoft.Json;

public class DirectRobotConnection : MonoBehaviour
{
    [Header("Direct Connection Settings")]
    public string serverUrl = "ws://localhost:8080";
    public float updateRate = 0.05f; // 20 Hz update rate

    private WebSocket webSocket;

    void Start()
    {
        StartCoroutine(ConnectWebSocket());
    }

    IEnumerator ConnectWebSocket()
    {
        webSocket = new WebSocket(new Uri(serverUrl));
        yield return StartCoroutine(webSocket.Connect());

        StartCoroutine(ReceiveData());
        StartCoroutine(SendHeartbeat());
    }

    IEnumerator ReceiveData()
    {
        while (webSocket.State == WebSocketState.Open)
        {
            if (webSocket.MessageAvailable)
            {
                string message = webSocket.RecvString();
                ProcessRobotData(message);
            }
            yield return new WaitForSeconds(0.01f);
        }
    }

    void ProcessRobotData(string jsonData)
    {
        try
        {
            RobotData robotData = JsonConvert.DeserializeObject<RobotData>(jsonData);
            UpdateRobotVisualization(robotData);
        }
        catch (System.Exception e)
        {
            Debug.LogError("Error processing robot data: " + e.Message);
        }
    }

    [System.Serializable]
    public class RobotData
    {
        public string robotId;
        public float timestamp;
        public Dictionary<string, float> jointPositions;
        public float[] position; // [x, y, z]
        public float[] orientation; // [x, y, z, w] quaternion
        public Dictionary<string, float[]> sensorData;
    }

    void UpdateRobotVisualization(RobotData data)
    {
        // Update robot model based on received data
        foreach (var joint in data.jointPositions)
        {
            Transform jointTransform = transform.Find(joint.Key);
            if (jointTransform != null)
            {
                jointTransform.localRotation = Quaternion.Euler(0, 0, joint.Value * Mathf.Rad2Deg);
            }
        }
    }

    IEnumerator SendHeartbeat()
    {
        while (webSocket.State == WebSocketState.Open)
        {
            var heartbeat = new { type = "heartbeat", timestamp = Time.time };
            webSocket.SendString(JsonConvert.SerializeObject(heartbeat));
            yield return new WaitForSeconds(5.0f); // Send heartbeat every 5 seconds
        }
    }
}
```

## Human-Robot Interaction Design Principles

Effective Human-Robot Interaction (HRI) design is crucial for digital twin applications, especially for complex humanoid robots. Well-designed interfaces can significantly improve operator efficiency and safety.

### Visual Feedback Systems

Visual feedback systems provide operators with immediate information about robot state and behavior:

#### Status Indicators
- **Operational Status**: Use color-coded indicators (green = operational, yellow = caution, red = error)
- **Battery Level**: Clear visualization of remaining power with predictive estimates
- **Computational Load**: Display CPU/GPU usage to anticipate performance issues
- **Communication Status**: Show connection quality and data throughput

#### Interaction Zones
- **Safe Interaction Areas**: Clearly marked zones where humans can safely approach the robot
- **No-Go Zones**: Visual barriers around sensitive areas or moving parts
- **Approach Guidelines**: Directional indicators for safe human-robot interaction

#### Gesture Recognition Feedback
```csharp
public class GestureFeedback : MonoBehaviour
{
    [Header("Gesture Recognition UI")]
    public GameObject gestureIndicator;
    public Material recognizedMaterial;
    public Material processingMaterial;
    public Material failedMaterial;

    private Renderer indicatorRenderer;

    void Start()
    {
        indicatorRenderer = gestureIndicator.GetComponent<Renderer>();
    }

    public void UpdateGestureStatus(GestureStatus status)
    {
        switch (status)
        {
            case GestureStatus.Recognized:
                indicatorRenderer.material = recognizedMaterial;
                break;
            case GestureStatus.Processing:
                indicatorRenderer.material = processingMaterial;
                StartCoroutine(PulseEffect());
                break;
            case GestureStatus.Failed:
                indicatorRenderer.material = failedMaterial;
                StartCoroutine(ShakeEffect());
                break;
        }
    }

    IEnumerator PulseEffect()
    {
        float originalScale = gestureIndicator.transform.localScale.x;
        for (int i = 0; i < 3; i++)
        {
            gestureIndicator.transform.localScale = Vector3.one * originalScale * 1.2f;
            yield return new WaitForSeconds(0.1f);
            gestureIndicator.transform.localScale = Vector3.one * originalScale;
            yield return new WaitForSeconds(0.1f);
        }
    }

    IEnumerator ShakeEffect()
    {
        Vector3 originalPosition = gestureIndicator.transform.position;
        for (int i = 0; i < 5; i++)
        {
            gestureIndicator.transform.position = originalPosition + Random.insideUnitSphere * 0.02f;
            yield return new WaitForSeconds(0.05f);
        }
        gestureIndicator.transform.position = originalPosition;
    }
}

public enum GestureStatus
{
    Processing,
    Recognized,
    Failed
}
```

### User Interface Elements for Robotics

#### Control Panels
Design intuitive interfaces for robot operation:

```csharp
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class RobotControlPanel : MonoBehaviour
{
    [Header("Control Panel Elements")]
    public Slider velocitySlider;
    public Toggle autonomousModeToggle;
    public Button[] actionButtons;
    public Text statusText;
    public Image progressBar;

    [Header("Robot Commands")]
    public RobotCommand[] availableCommands;

    private RobotController robotController;
    private Dictionary<string, Button> commandButtons = new Dictionary<string, Button>();

    void Start()
    {
        InitializeControlPanel();
        SetupEventHandlers();
    }

    void InitializeControlPanel()
    {
        // Initialize velocity slider
        velocitySlider.minValue = 0.1f;
        velocitySlider.maxValue = 1.0f;
        velocitySlider.value = 0.5f;

        // Create command buttons dynamically
        foreach (var command in availableCommands)
        {
            CreateCommandButton(command);
        }
    }

    void SetupEventHandlers()
    {
        velocitySlider.onValueChanged.AddListener(OnVelocityChanged);
        autonomousModeToggle.onValueChanged.AddListener(OnAutonomousModeChanged);
    }

    void CreateCommandButton(RobotCommand command)
    {
        GameObject buttonObj = new GameObject(command.name + " Button");
        buttonObj.transform.SetParent(transform);
        Button button = buttonObj.AddComponent<Button>();
        Text buttonText = buttonObj.AddComponent<Text>();
        buttonText.text = command.displayName;

        button.onClick.AddListener(() => ExecuteCommand(command));
        commandButtons[command.name] = button;
    }

    void OnVelocityChanged(float value)
    {
        if (robotController != null)
        {
            robotController.SetVelocityScale(value);
        }
    }

    void OnAutonomousModeChanged(bool isAutonomous)
    {
        if (robotController != null)
        {
            robotController.SetAutonomousMode(isAutonomous);
        }
    }

    void ExecuteCommand(RobotCommand command)
    {
        if (robotController != null)
        {
            robotController.ExecuteCommand(command);
            UpdateStatus("Executing: " + command.displayName);
        }
    }

    void UpdateStatus(string message)
    {
        statusText.text = message;
        StartCoroutine(ClearStatusAfterDelay(3.0f));
    }

    IEnumerator ClearStatusAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        if (statusText.text.StartsWith("Executing: "))
        {
            statusText.text = "Ready";
        }
    }
}

[System.Serializable]
public class RobotCommand
{
    public string name;
    public string displayName;
    public string description;
    public CommandType type;
    public float executionTime;
}

public enum CommandType
{
    Movement,
    Manipulation,
    Sensing,
    Communication,
    Emergency
}
```

#### Monitoring Displays
Real-time visualization of robot state and sensor data:

- **Joint Position Monitors**: Visual indicators showing current joint angles
- **Sensor Data Displays**: Real-time visualization of LiDAR, camera, and IMU data
- **Path Planning Visualization**: Show planned and executed trajectories
- **Force/Torque Feedback**: Visual representation of interaction forces

#### Safety Indicators
Critical for preventing accidents and ensuring safe operation:

- **Emergency Stop Visualization**: Clear, prominent emergency stop indicators
- **Collision Avoidance Status**: Real-time display of collision prediction systems
- **Safety Zone Monitoring**: Visual representation of safety perimeters
- **Warning Systems**: Progressive warning indicators before critical situations

### Interaction Patterns for Humanoid Robots

#### Direct Manipulation
Allowing users to interact with robot elements directly in the 3D space:

```csharp
public class DirectManipulation : MonoBehaviour
{
    [Header("Manipulation Settings")]
    public LayerMask interactionLayer;
    public float interactionDistance = 5.0f;
    public GameObject interactionCursor;

    private Camera mainCamera;
    private GameObject currentInteractionTarget;
    private bool isInteracting = false;

    void Start()
    {
        mainCamera = Camera.main;
    }

    void Update()
    {
        HandleDirectManipulation();
    }

    void HandleDirectManipulation()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, interactionDistance, interactionLayer))
            {
                StartInteraction(hit.collider.gameObject);
            }
        }
        else if (Input.GetMouseButton(0) && isInteracting)
        {
            ContinueInteraction();
        }
        else if (Input.GetMouseButtonUp(0))
        {
            EndInteraction();
        }
    }

    void StartInteraction(GameObject target)
    {
        currentInteractionTarget = target;
        isInteracting = true;

        // Highlight the target
        HighlightTarget(target, true);

        // Send command to robot if needed
        SendManipulationCommand(target);
    }

    void ContinueInteraction()
    {
        if (currentInteractionTarget != null)
        {
            // Update interaction based on mouse movement
            UpdateTargetPosition();
        }
    }

    void EndInteraction()
    {
        if (currentInteractionTarget != null)
        {
            HighlightTarget(currentInteractionTarget, false);
            currentInteractionTarget = null;
        }
        isInteracting = false;
    }

    void HighlightTarget(GameObject target, bool highlight)
    {
        Renderer renderer = target.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material.color = highlight ? Color.yellow : Color.white;
        }
    }

    void SendManipulationCommand(GameObject target)
    {
        // Send command to robot to interact with the target
        string targetName = target.name;
        // Implementation to send command to robot via ROS or direct communication
    }

    void UpdateTargetPosition()
    {
        // Update the target position based on cursor movement
        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, interactionDistance, interactionLayer))
        {
            currentInteractionTarget.transform.position = hit.point;
        }
    }
}
```

#### Remote Control Interface
For operation from a safe distance:

- **Teleoperation Controls**: Gamepad or keyboard-based control systems
- **Virtual Cockpit**: Immersive interface showing robot perspective
- **Gesture-Based Control**: Using camera or VR controllers for intuitive operation

#### Supervisory Control
High-level command interfaces:

- **Task Planning**: Interface for defining complex multi-step tasks
- **Behavior Selection**: Choosing from predefined robot behaviors
- **Goal-Based Control**: Setting destinations and objectives rather than low-level commands

### Accessibility and Usability Considerations

#### Universal Design Principles
- **Clear Visual Hierarchy**: Organize information by importance
- **Consistent Interaction Patterns**: Use familiar interface metaphors
- **Multiple Feedback Channels**: Combine visual, auditory, and haptic feedback
- **Error Prevention**: Design to minimize operator errors
- **Recovery Options**: Provide clear paths to recover from errors

#### Performance Optimization
- **Responsive Interfaces**: Ensure UI elements respond quickly to input
- **Efficient Rendering**: Optimize graphics for real-time interaction
- **Network Efficiency**: Minimize latency in remote operation scenarios

## VR/AR Integration for Enhanced HRI

Virtual and Augmented Reality technologies provide unprecedented opportunities for intuitive and immersive human-robot interaction, particularly for complex humanoid robots.

### Virtual Reality Integration

Unity's VR capabilities enable fully immersive robot interaction experiences that can significantly improve operator understanding and control:

```csharp
using UnityEngine;
using UnityEngine.XR;
using System.Collections;

public class VRInteraction : MonoBehaviour
{
    [Header("VR Interaction Settings")]
    public GameObject robot;
    public XRNode leftControllerNode;
    public XRNode rightControllerNode;
    public LayerMask interactionLayer;
    public float interactionDistance = 3.0f;

    private InputDevice leftController;
    private InputDevice rightController;
    private GameObject grabbedObject;
    private bool isLeftGripping = false;
    private bool isRightGripping = false;

    void Start()
    {
        InitializeVRControllers();
    }

    void Update()
    {
        UpdateControllerStates();
        HandleVRInteractions();
    }

    void InitializeVRControllers()
    {
        leftController = InputDevices.GetDeviceAtXRNode(leftControllerNode);
        rightController = InputDevices.GetDeviceAtXRNode(rightControllerNode);
    }

    void UpdateControllerStates()
    {
        // Update left controller state
        leftController = InputDevices.GetDeviceAtXRNode(leftControllerNode);
        if (leftController.isValid)
        {
            UpdateControllerInput(leftController, true);
        }

        // Update right controller state
        rightController = InputDevices.GetDeviceAtXRNode(rightControllerNode);
        if (rightController.isValid)
        {
            UpdateControllerInput(rightController, false);
        }
    }

    void UpdateControllerInput(InputDevice controller, bool isLeftController)
    {
        // Get trigger and grip button states
        controller.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerPressed);
        controller.TryGetFeatureValue(CommonUsages.gripButton, out bool gripPressed);

        // Update grip state
        if (isLeftController)
        {
            isLeftGripping = gripPressed;
        }
        else
        {
            isRightGripping = gripPressed;
        }

        // Handle object grabbing
        if (gripPressed && grabbedObject == null)
        {
            TryGrabObject(isLeftController ? leftController : rightController, isLeftController);
        }
        else if (!gripPressed && grabbedObject != null)
        {
            ReleaseObject(isLeftController);
        }

        // Handle other interactions
        if (triggerPressed)
        {
            PerformVRInteraction(isLeftController ? leftController : rightController, isLeftController);
        }
    }

    void TryGrabObject(InputDevice controller, bool isLeftController)
    {
        // Raycast from controller to find grabbable objects
        Vector3 controllerPosition = GetControllerPosition(controller);
        Vector3 controllerForward = GetControllerForward(controller);

        RaycastHit hit;
        if (Physics.Raycast(controllerPosition, controllerForward, out hit, interactionDistance, interactionLayer))
        {
            if (hit.collider.CompareTag("Grabbable") || hit.collider.CompareTag("RobotPart"))
            {
                grabbedObject = hit.collider.gameObject;

                // Make object a child of the controller
                grabbedObject.transform.SetParent(isLeftController ?
                    transform.Find("LeftController") : transform.Find("RightController"));

                // Disable physics while grabbed
                Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.isKinematic = true;
                }

                Debug.Log("Grabbed: " + grabbedObject.name);
            }
        }
    }

    void ReleaseObject(bool wasLeftController)
    {
        if (grabbedObject != null)
        {
            // Release the object
            grabbedObject.transform.SetParent(null);

            // Re-enable physics
            Rigidbody rb = grabbedObject.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = false;

                // Apply velocity based on controller movement
                Vector3 controllerVelocity = GetControllerVelocity(wasLeftController ?
                    leftController : rightController);
                rb.velocity = controllerVelocity;
            }

            grabbedObject = null;
            Debug.Log("Released object");
        }
    }

    void PerformVRInteraction(InputDevice controller, bool isLeftController)
    {
        // Perform interaction based on trigger press
        Vector3 controllerPosition = GetControllerPosition(controller);
        Vector3 controllerForward = GetControllerForward(controller);

        RaycastHit hit;
        if (Physics.Raycast(controllerPosition, controllerForward, out hit, interactionDistance, interactionLayer))
        {
            if (hit.collider.CompareTag("RobotPart"))
            {
                // Send interaction command to robot
                RobotPart robotPart = hit.collider.GetComponent<RobotPart>();
                if (robotPart != null)
                {
                    robotPart.OnInteract();
                    SendRobotCommand(robotPart.name, "interact");
                }
            }
            else if (hit.collider.CompareTag("UIElement"))
            {
                // Interact with UI elements
                UIElement uiElement = hit.collider.GetComponent<UIElement>();
                if (uiElement != null)
                {
                    uiElement.OnClick();
                }
            }
        }
    }

    Vector3 GetControllerPosition(InputDevice controller)
    {
        controller.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
        return position;
    }

    Vector3 GetControllerForward(InputDevice controller)
    {
        controller.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);
        return rotation * Vector3.forward;
    }

    Vector3 GetControllerVelocity(InputDevice controller)
    {
        controller.TryGetFeatureValue(CommonUsages.deviceVelocity, out Vector3 velocity);
        return velocity;
    }

    void HandleVRInteractions()
    {
        // Additional VR-specific interaction logic
        // For example, gesture recognition using controller movements
    }

    void SendRobotCommand(string target, string command)
    {
        // Implementation to send command to robot via ROS or direct communication
        Debug.Log($"Sending command '{command}' to {target}");
    }
}

// Component for robot parts that can be interacted with in VR
public class RobotPart : MonoBehaviour
{
    [Header("Robot Part Settings")]
    public string partName;
    public bool isInteractive = true;
    public Color originalColor;
    public Color highlightColor = Color.yellow;

    private Renderer partRenderer;
    private bool isHighlighted = false;

    void Start()
    {
        partRenderer = GetComponent<Renderer>();
        if (partRenderer != null)
        {
            originalColor = partRenderer.material.color;
        }
    }

    public void OnInteract()
    {
        if (!isInteractive) return;

        Debug.Log($"Interacting with robot part: {partName}");

        // Visual feedback for interaction
        StartCoroutine(InteractionFeedback());
    }

    public void SetHighlight(bool highlight)
    {
        if (!isInteractive || partRenderer == null) return;

        isHighlighted = highlight;
        partRenderer.material.color = highlight ? highlightColor : originalColor;
    }

    IEnumerator InteractionFeedback()
    {
        // Visual feedback animation
        Material originalMaterial = partRenderer.material;

        // Flash the part
        for (int i = 0; i < 3; i++)
        {
            partRenderer.material.color = Color.red;
            yield return new WaitForSeconds(0.1f);
            partRenderer.material.color = originalColor;
            yield return new WaitForSeconds(0.1f);
        }

        // Return to original color
        partRenderer.material = originalMaterial;
    }
}
```

### Advanced VR Interaction Patterns

#### Haptic Feedback Integration
```csharp
using UnityEngine.XR;

public class HapticFeedback : MonoBehaviour
{
    [Header("Haptic Settings")]
    public XRNode controllerNode;
    public float defaultAmplitude = 0.5f;
    public float duration = 0.1f;

    private InputDevice controller;

    void Start()
    {
        controller = InputDevices.GetDeviceAtXRNode(controllerNode);
    }

    public void TriggerHapticFeedback(float amplitude = -1)
    {
        if (amplitude < 0) amplitude = defaultAmplitude;

        if (controller.isValid)
        {
            controller.SendHapticImpulse(0, amplitude, duration);
        }
    }

    public void TriggerHapticFeedback(float amplitude, float duration)
    {
        if (controller.isValid)
        {
            controller.SendHapticImpulse(0, amplitude, duration);
        }
    }
}
```

### Augmented Reality Applications

AR technology overlays digital information onto the real world, creating powerful interfaces for robot operation and monitoring:

#### AR Robot State Visualization
```csharp
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using System.Collections.Generic;

public class ARRobotOverlay : MonoBehaviour
{
    [Header("AR Overlay Settings")]
    public GameObject robotModel;
    public GameObject statusOverlay;
    public GameObject trajectoryDisplay;
    public Camera arCamera;

    private ARSession arSession;
    private ARRaycastManager raycastManager;
    private Vector2 touchPosition;

    void Start()
    {
        arSession = FindObjectOfType<ARSession>();
        raycastManager = FindObjectOfType<ARRaycastManager>();
    }

    void Update()
    {
        HandleARInput();
        UpdateRobotOverlay();
    }

    void HandleARInput()
    {
        if (Input.touchCount > 0)
        {
            Touch touch = Input.GetTouch(0);
            touchPosition = touch.position;

            if (touch.phase == TouchPhase.Began)
            {
                ARRaycast();
            }
        }
    }

    void ARRaycast()
    {
        List<ARRaycastHit> hits = new List<ARRaycastHit>();
        if (raycastManager.Raycast(touchPosition, hits, TrackableType.Planes))
        {
            Pose hitPose = hits[0].pose;

            // Place robot visualization at the hit position
            PlaceRobotVisualization(hitPose);
        }
    }

    void PlaceRobotVisualization(Pose pose)
    {
        // Instantiate or move robot visualization to the AR position
        if (robotModel != null)
        {
            robotModel.transform.SetPositionAndRotation(pose.position, pose.rotation);
        }
    }

    void UpdateRobotOverlay()
    {
        // Update status overlay with real-time robot data
        if (statusOverlay != null)
        {
            UpdateStatusInformation();
        }

        // Update trajectory display
        if (trajectoryDisplay != null)
        {
            UpdateTrajectoryVisualization();
        }
    }

    void UpdateStatusInformation()
    {
        // Update with real-time robot status
        // This could include battery level, operational status, etc.
    }

    void UpdateTrajectoryVisualization()
    {
        // Visualize planned and executed robot trajectories
        // This could be done with line renderers or particle systems
    }
}
```

#### AR-Based Robot Programming
AR interfaces can enable intuitive robot programming by allowing users to demonstrate desired behaviors in the real environment:

- **Path Recording**: Demonstrate desired robot paths by moving a controller in space
- **Object Recognition Training**: Point at objects to teach the robot what they are
- **Gesture Teaching**: Perform gestures to teach the robot new behaviors
- **Environment Mapping**: Mark important locations and areas in the robot's workspace

### Performance Optimization for VR/AR

#### Quality Settings for Real-Time Performance
```csharp
using UnityEngine;

public class VRPerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Settings")]
    public int targetFrameRate = 90; // For VR applications
    public LODGroup robotLOD;
    public float maxRenderDistance = 10.0f;

    void Start()
    {
        OptimizeForVR();
    }

    void OptimizeForVR()
    {
        // Set target frame rate
        Application.targetFrameRate = targetFrameRate;

        // Optimize quality settings
        QualitySettings.vSyncCount = 0;
        QualitySettings.maxQueuedFrames = 2;

        // Optimize robot rendering based on distance
        StartCoroutine(OptimizeRobotLOD());
    }

    IEnumerator OptimizeRobotLOD()
    {
        while (true)
        {
            if (Vector3.Distance(Camera.main.transform.position, transform.position) > maxRenderDistance)
            {
                // Use lower LOD for distant robots
                robotLOD.ForceLOD(2);
            }
            else
            {
                // Use appropriate LOD based on distance
                float distance = Vector3.Distance(Camera.main.transform.position, transform.position);
                int lodLevel = Mathf.FloorToInt(distance / (maxRenderDistance / 3));
                robotLOD.ForceLOD(Mathf.Clamp(lodLevel, 0, 2));
            }

            yield return new WaitForSeconds(0.5f); // Update LOD every 0.5 seconds
        }
    }
}
```

### Best Practices for VR/AR HRI

#### Safety Considerations
- **Physical Safety**: Ensure VR users are aware of real-world obstacles
- **Motion Sickness**: Minimize latency and maintain high frame rates
- **Emergency Procedures**: Provide quick exit mechanisms from VR experiences
- **User Monitoring**: Implement systems to observe VR users for safety

#### Usability Guidelines
- **Intuitive Mapping**: Ensure virtual controls map naturally to robot actions
- **Clear Feedback**: Provide immediate and clear feedback for all interactions
- **Progressive Disclosure**: Show complex information only when needed
- **Consistent Metaphors**: Use consistent interaction patterns throughout the application

## Exercises

### Exercise 1: Unity Robot Visualization Setup
Implement a complete robot visualization system in Unity:
1. Create a humanoid robot model with appropriate joint hierarchy
2. Set up URP rendering pipeline for optimal performance
3. Implement joint mapping between ROS joint names and Unity transforms
4. Add visual indicators for joint limits and current positions
5. Test with simulated robot data to ensure smooth visualization

### Exercise 2: Real-time Synchronization Implementation
Build a synchronization system between Gazebo and Unity:
1. Set up ROS# connection to receive joint states from Gazebo
2. Implement interpolation for smooth joint movement visualization
3. Add error handling for connection failures
4. Create a status indicator showing connection quality
5. Test with a simulated humanoid robot in Gazebo

### Exercise 3: Human-Robot Interaction Interface
Design and implement an HRI interface for robot control:
1. Create a control panel with buttons for common robot commands
2. Implement direct manipulation for robot parts in the 3D view
3. Add safety indicators and emergency stop functionality
4. Create a monitoring display showing robot state and sensor data
5. Test the interface with simulated robot responses

### Exercise 4: VR/AR Integration
Implement immersive interaction using VR/AR technologies:
1. Set up VR controllers for robot interaction in Unity
2. Implement object grabbing and manipulation using VR controllers
3. Add haptic feedback for enhanced interaction
4. Create an AR overlay showing robot status in a mobile application
5. Test the VR interface for performance and usability

## Performance Optimization

### Rendering Optimization
- **Level of Detail (LOD)**: Adjust detail based on distance
- **Occlusion Culling**: Hide objects not visible to camera
- **Texture Streaming**: Load textures as needed
- **Shader Optimization**: Use efficient shaders for real-time rendering

### Memory Management
- **Object Pooling**: Reuse objects instead of creating new ones
- **Asset Bundles**: Load assets dynamically as needed
- **Garbage Collection**: Minimize allocation to reduce GC pauses

## Exercises

1. **Unity Robot Model**: Create a humanoid robot model in Unity with proper joint hierarchy
2. **Synchronization**: Implement basic synchronization between simulated robot states and Unity visualization
3. **HRI Interface**: Design and implement a basic human-robot interaction interface in Unity

## Summary

This chapter covered Unity's capabilities for digital twin creation and human-robot interaction. We explored rendering pipelines, synchronization with Gazebo, HRI design principles, and VR/AR integration. The next chapter will focus on sensor simulation and validation.