---
sidebar_position: 3
title: Sensor Simulation & Validation
---

# Sensor Simulation & Validation

## Introduction to Sensor Simulation in Robotics

This chapter covers the simulation of various sensors commonly used in humanoid robotics, including LiDAR, depth cameras, and IMUs. Proper sensor simulation is crucial for developing and validating perception algorithms in a safe, controlled environment before deployment on real robots.

### The Importance of Sensor Simulation

Sensor simulation plays a critical role in robotics development for several key reasons:

1. **Safe Algorithm Development**: Test perception and navigation algorithms without risk to expensive hardware
2. **Cost-Effective Training**: Train machine learning models on large datasets of simulated sensor data
3. **Controlled Testing**: Create reproducible test scenarios with known ground truth
4. **Edge Case Exploration**: Generate rare or dangerous scenarios safely in simulation
5. **Hardware-in-the-Loop Testing**: Validate sensor fusion algorithms before deployment

For humanoid robots specifically, sensor simulation is essential because these robots operate in human environments where safety is paramount and physical testing can be extremely costly.

### Sensor Types for Humanoid Robotics

Humanoid robots typically use a combination of the following sensor types:

- **LiDAR**: Provides 2D or 3D range data for mapping and navigation
- **Depth Cameras**: RGB-D sensors for environment perception and object recognition
- **IMUs**: Inertial measurement units for balance, orientation, and motion detection
- **Force/Torque Sensors**: Located in joints or feet for contact detection and balance
- **Tactile Sensors**: For fine manipulation and contact feedback
- **Cameras**: Visual information for object recognition and scene understanding

### Simulation Fidelity Requirements

The fidelity required for sensor simulation depends on the intended application:

- **Algorithm Development**: High fidelity to ensure transferability to real robots
- **Training Data Generation**: Sufficient realism to be useful for machine learning
- **Safety Validation**: Accurate modeling of sensor limitations and failure modes
- **Performance Evaluation**: Realistic noise models and response characteristics

### Learning Objectives
- Understand LiDAR simulation with realistic noise models
- Learn about depth camera simulation and point cloud generation
- Master IMU simulation with realistic drift and noise characteristics
- Explore sensor fusion techniques in simulation environments
- Validate sensor simulation accuracy against real-world data
- Compare simulated vs. real sensor data for algorithm validation
- Implement sensor fusion algorithms for improved state estimation

### Prerequisites
- Understanding of basic sensor types and their applications
- Knowledge of physics simulation from Chapter 1
- Familiarity with digital twin concepts from Chapter 2

## LiDAR Simulation

Light Detection and Ranging (LiDAR) sensors are crucial for humanoid robots, providing accurate 2D or 3D spatial information for navigation, mapping, and obstacle detection.

### LiDAR Physics and Characteristics

LiDAR sensors work by emitting laser pulses and measuring the time it takes for the light to return after reflecting off objects. Key characteristics include:

- **Range**: Distance measurement capability (typically 0.1m to 30m for robotics)
- **Field of View**: Angular coverage (horizontal and vertical)
- **Resolution**: Angular resolution and distance accuracy
- **Update Rate**: How frequently measurements are taken (typically 5-20 Hz)
- **Accuracy**: Measurement precision and repeatability

### Gazebo LiDAR Plugin

Gazebo provides a robust LiDAR sensor plugin that simulates 2D and 3D LiDAR sensors with realistic characteristics:

```xml
<gazebo reference="lidar_mount">
  <sensor type="ray" name="humanoid_lidar_2d">
    <pose>0 0 0.5 0 0 0</pose> <!-- Mount at 0.5m height -->
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples> <!-- 0.5 degree resolution over 360 degrees -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -π radians (-180 degrees) -->
          <max_angle>3.14159</max_angle>  <!-- π radians (180 degrees) -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min> <!-- Minimum detectable range: 0.1m -->
        <max>30.0</max> <!-- Maximum detectable range: 30m -->
        <resolution>0.01</resolution> <!-- Range resolution: 1cm -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topicName>/humanoid_robot/laser_scan</topicName>
      <frameName>lidar_mount</frameName>
      <min_range>0.1</min_range>
      <max_range>30.0</max_range>
      <gaussian_noise>0.01</gaussian_noise> <!-- 1cm noise standard deviation -->
    </plugin>
  </sensor>
</gazebo>
```

### 3D LiDAR Configuration

For humanoid robots requiring full 3D perception, a 3D LiDAR configuration is essential:

```xml
<gazebo reference="lidar_mount">
  <sensor type="gpu_ray" name="humanoid_lidar_3d">
    <pose>0 0 0.5 0 0 0</pose>
    <visualize>false</visualize> <!-- Disable visualization for performance -->
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples> <!-- Horizontal samples -->
          <resolution>1</resolution>
          <min_angle>-1.3962634</min_angle> <!-- -80 degrees -->
          <max_angle>1.3962634</max_angle>  <!-- 80 degrees -->
        </horizontal>
        <vertical>
          <samples>32</samples> <!-- Vertical channels -->
          <resolution>1</resolution>
          <min_angle>-0.261799387</min_angle> <!-- -15 degrees -->
          <max_angle>0.261799387</max_angle>  <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.08</min> <!-- Minimum range -->
        <max>100.0</max> <!-- Maximum range -->
        <resolution>0.01</resolution> <!-- Range resolution -->
      </range>
    </ray>
    <plugin name="velodyne_controller" filename="libgazebo_ros_velodyne_gpu_laser.so">
      <topicName>/humanoid_robot/velodyne_points</topicName>
      <frameName>lidar_mount</frameName>
      <min_range>0.9</min_range>
      <max_range>100.0</max_range>
      <gaussian_noise>0.01</gaussian_noise>
    </plugin>
  </sensor>
</gazebo>
```

### Realistic LiDAR Noise Modeling

Realistic noise modeling is crucial for transfer learning from simulation to reality:

- **Gaussian Noise**: Random measurement errors
- **Bias**: Systematic measurement offsets
- **Drift**: Time-varying systematic errors
- **Dropout**: Occasional complete measurement failures
- **Multi-path Interference**: Reflections causing false readings

```xml
<!-- Advanced noise modeling for realistic LiDAR simulation -->
<sensor type="ray" name="advanced_lidar">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>25.0</max>
      <resolution>0.001</resolution>
    </range>
  </ray>
  <plugin name="advanced_lidar_plugin" filename="libgazebo_ros_laser.so">
    <topicName>/humanoid_robot/advanced_laser_scan</topicName>
    <frameName>lidar_mount</frameName>
    <!-- Noise parameters -->
    <gaussian_noise>0.008</gaussian_noise> <!-- 8mm noise -->
    <bias_mean>0.002</bias_mean> <!-- 2mm bias -->
    <bias_stddev>0.001</bias_stddev> <!-- 1mm bias variation -->
    <!-- Range-dependent noise -->
    <return_phase_correction>0.0</return_phase_correction>
    <signal_size>0.0</signal_size>
    <reference_coordinates>0</reference_coordinates>
  </plugin>
</sensor>
```

### LiDAR Performance Optimization

For humanoid robots with limited computational resources:

- **Reduced Resolution**: Lower sample counts for faster processing
- **Limited FOV**: Focus on relevant directions only
- **Variable Update Rate**: Adjust based on robot speed and environment complexity
- **Selective Visualization**: Only visualize when debugging

### Point Cloud Processing in Simulation

3D LiDAR sensors generate point clouds that require specialized processing:

```python
#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point32

class LiDARProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor')
        self.subscriber = rospy.Subscriber('/humanoid_robot/velodyne_points',
                                          PointCloud2, self.point_cloud_callback)
        self.ground_publisher = rospy.Publisher('/humanoid_robot/ground_points',
                                               PointCloud2, queue_size=1)
        self.obstacle_publisher = rospy.Publisher('/humanoid_robot/obstacle_points',
                                                 PointCloud2, queue_size=1)

    def point_cloud_callback(self, msg):
        """
        Process incoming point cloud data from simulated LiDAR
        """
        # Convert PointCloud2 to list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Convert to numpy array for processing
        points_array = np.array(points)

        if len(points_array) == 0:
            return

        # Segment ground plane using RANSAC
        ground_points, obstacle_points = self.segment_ground_plane(points_array)

        # Publish segmented point clouds
        self.publish_point_cloud(ground_points, self.ground_publisher, msg.header)
        self.publish_point_cloud(obstacle_points, self.obstacle_publisher, msg.header)

    def segment_ground_plane(self, points, distance_threshold=0.1, max_iterations=100):
        """
        Segment ground plane from point cloud using RANSAC
        """
        if len(points) < 100:  # Need minimum points for RANSAC
            return np.array([]), points

        best_model = None
        best_inliers = []

        for _ in range(max_iterations):
            # Randomly sample 3 points
            sample_indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[sample_indices]

            # Fit plane to 3 points
            plane_model = self.fit_plane(sample_points)

            if plane_model is None:
                continue

            # Find inliers
            distances = np.abs(np.dot(points - sample_points[0], plane_model[:3]) + plane_model[3])
            inliers = points[distances < distance_threshold]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = plane_model

        if best_model is not None:
            # Separate ground and obstacles
            ground_mask = np.abs(np.dot(points - best_inliers[0], best_model[:3]) + best_model[3]) < distance_threshold
            ground_points = points[ground_mask]
            obstacle_points = points[~ground_mask]
            return ground_points, obstacle_points
        else:
            # If RANSAC failed, return original points
            return np.array([]), points

    def fit_plane(self, points):
        """
        Fit a plane to 3 points: ax + by + cz + d = 0
        """
        if len(points) < 3:
            return None

        p1, p2, p3 = points[:3]

        # Calculate plane normal vector
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)

        if np.linalg.norm(normal) < 1e-6:  # Points are collinear
            return None

        normal = normal / np.linalg.norm(normal)
        d = -np.dot(normal, p1)

        return np.append(normal, d)

    def publish_point_cloud(self, points, publisher, header):
        """
        Publish point cloud to ROS topic
        """
        if len(points) == 0:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "lidar_frame"

        # Convert numpy array to PointCloud2
        cloud_points = [Point32(x, y, z) for x, y, z in points]
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)

        publisher.publish(cloud_msg)

if __name__ == '__main__':
    processor = LiDARProcessor()
    rospy.spin()
```

### Noise Models for LiDAR

Realistic LiDAR simulation includes various noise sources:

- **Gaussian Noise**: Random measurement errors
- **Bias**: Systematic measurement offsets
- **Drift**: Time-varying systematic errors

### Point Cloud Generation

3D LiDAR sensors generate point clouds that require special handling:

```xml
<sensor type="gpu_ray" name="3d_lidar">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>false</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.3962634</min_angle>
        <max_angle>1.3962634</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799387</min_angle>
        <max_angle>0.261799387</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

## Depth Camera Simulation

RGB-D cameras (color + depth) are essential sensors for humanoid robots, providing both visual information and depth data for object recognition, scene understanding, and navigation.

### Depth Camera Physics and Characteristics

Depth cameras work by measuring the distance to objects in the scene using various technologies:

- **Stereo Vision**: Uses two cameras to calculate depth from parallax
- **Structured Light**: Projects a known pattern and measures distortions
- **Time-of-Flight**: Measures the time light takes to return from projected pulses

Key characteristics include:

- **Resolution**: Image dimensions (e.g., 640x480, 1280x720)
- **Field of View**: Angular coverage (horizontal and vertical)
- **Depth Range**: Minimum and maximum measurable distances
- **Depth Accuracy**: Precision of distance measurements
- **Update Rate**: Frame rate (typically 15-30 Hz)

### Advanced RGB-D Camera Configuration

```xml
<gazebo reference="camera_mount">
  <sensor type="depth" name="humanoid_depth_camera_advanced">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <format>R8G8B8</format>
        <width>1280</width> <!-- Higher resolution for detail -->
        <height>720</height>
      </image>
      <clip>
        <near>0.1</near> <!-- Minimum range: 10cm -->
        <far>8.0</far>   <!-- Maximum range: 8m -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev> <!-- 7mm depth noise -->
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <!-- Camera parameters -->
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>head_camera</cameraName>

      <!-- Topic names -->
      <imageTopicName>rgb/image_raw</imageTopicName>
      <depthImageTopicName>depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>depth/points</pointCloudTopicName>
      <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>

      <!-- Frame configuration -->
      <frameName>camera_depth_optical_frame</frameName>
      <pointCloudCutoff>0.1</pointCloudCutoff>    <!-- Near cutoff -->
      <pointCloudCutoffMax>8.0</pointCloudCutoffMax> <!-- Far cutoff -->

      <!-- Distortion parameters -->
      <distortion_k1>0.001</distortion_k1>
      <distortion_k2>0.0005</distortion_k2>
      <distortion_k3>0.0001</distortion_k3>
      <distortion_t1>0.0002</distortion_t1>
      <distortion_t2>0.0003</distortion_t2>

      <!-- Intrinsic parameters -->
      <CxPrime>0.0</CxPrime>
      <Cx>640.0</Cx> <!-- Principal point X -->
      <Cy>360.0</Cy> <!-- Principal point Y -->
      <focalLength>640.0</focalLength> <!-- Focal length in pixels -->
      <hackBaseline>0.0</hackBaseline>
    </plugin>
  </sensor>
</gazebo>
```

### Realistic Depth Camera Noise Modeling

Realistic noise modeling is critical for depth cameras:

```xml
<!-- Depth camera with realistic noise model -->
<sensor type="depth" name="realistic_depth_camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <!-- Color channel noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>
    </noise>
  </camera>
  <plugin name="realistic_camera_plugin" filename="libgazebo_ros_openni_kinect.so">
    <cameraName>realistic_camera</cameraName>
    <!-- Advanced noise parameters -->
    <depthImageTopicName>depth/image_raw_noisy</depthImageTopicName>
    <!-- Depth-dependent noise -->
    <pointCloudCutoff>0.3</pointCloudCutoff>
    <pointCloudCutoffMax>5.0</pointCloudCutoffMax>
    <!-- Add more realistic distortion -->
    <distortion_k1>0.0012</distortion_k1>
    <distortion_k2>-0.0003</distortion_k2>
    <distortion_k3>0.0001</distortion_k3>
    <distortion_t1>0.0001</distortion_t1>
    <distortion_t2>-0.0002</distortion_t2>
  </plugin>
</sensor>
```

### Depth Camera Processing Pipeline

```python
#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_euler
import message_filters

class DepthCameraProcessor:
    def __init__(self):
        rospy.init_node('depth_camera_processor')

        # Initialize bridge for image conversion
        self.bridge = CvBridge()

        # Synchronize RGB and depth images
        rgb_sub = message_filters.Subscriber('/humanoid_robot/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/humanoid_robot/depth/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        ts.registerCallback(self.synchronized_callback)

        # Publishers for processed data
        self.object_detection_pub = rospy.Publisher('/humanoid_robot/object_detections',
                                                   PointCloud2, queue_size=1)
        self.surface_normal_pub = rospy.Publisher('/humanoid_robot/surface_normals',
                                                 PointCloud2, queue_size=1)

        # Camera intrinsics (will be updated from camera_info)
        self.fx = 640.0  # Focal length x
        self.fy = 640.0  # Focal length y
        self.cx = 640.0  # Principal point x
        self.cy = 360.0  # Principal point y

    def synchronized_callback(self, rgb_msg, depth_msg):
        """
        Process synchronized RGB and depth images
        """
        try:
            # Convert ROS images to OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Process the images
            object_points = self.detect_objects(rgb_image, depth_image)
            surface_normals = self.calculate_surface_normals(depth_image)

            # Publish results
            if object_points.size > 0:
                self.publish_point_cloud(object_points, self.object_detection_pub, rgb_msg.header)

            if surface_normals.size > 0:
                self.publish_point_cloud(surface_normals, self.surface_normal_pub, rgb_msg.header)

        except Exception as e:
            rospy.logerr(f"Error processing synchronized images: {e}")

    def detect_objects(self, rgb_image, depth_image):
        """
        Simple object detection using color and depth information
        """
        # Convert BGR to HSV for better color detection
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for object detection (example: red objects)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Apply depth mask to focus on objects at certain distances
        depth_mask = (depth_image > 0.5) & (depth_image < 3.0)  # 50cm to 3m
        combined_mask = mask & depth_mask

        # Find contours
        contours, _ = cv2.findContours(combined_mask.astype(np.uint8),
                                      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_points = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small contours
                # Get the center of the contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Get depth at this point
                    depth_val = depth_image[cY, cX]
                    if depth_val > 0:  # Valid depth
                        # Convert to 3D point
                        x = (cX - self.cx) * depth_val / self.fx
                        y = (cY - self.cy) * depth_val / self.fy
                        z = depth_val

                        object_points.append([x, y, z])

        return np.array(object_points)

    def calculate_surface_normals(self, depth_image):
        """
        Calculate surface normals from depth image
        """
        # Calculate gradients
        zx = cv2.Sobel(depth_image, cv2.CV_64F, 1, 0, ksize=3)
        zy = cv2.Sobel(depth_image, cv2.CV_64F, 0, 1, ksize=3)

        # Calculate surface normals
        # For depth image, normal = (-dz/dx, -dz/dy, 1)
        normals = np.zeros((depth_image.shape[0], depth_image.shape[1], 3))
        normals[:, :, 0] = -zx  # -dz/dx
        normals[:, :, 1] = -zy  # -dz/dy
        normals[:, :, 2] = 1.0  # dz/dz = 1

        # Normalize
        norm = np.linalg.norm(normals, axis=2, keepdims=True)
        norm = np.where(norm == 0, 1, norm)  # Avoid division by zero
        normals = normals / norm

        # Sample normals for visualization
        sample_step = 20
        sampled_normals = []

        for y in range(0, depth_image.shape[0], sample_step):
            for x in range(0, depth_image.shape[1], sample_step):
                if depth_image[y, x] > 0:  # Valid depth
                    # Convert to 3D point
                    point_x = (x - self.cx) * depth_image[y, x] / self.fx
                    point_y = (y - self.cy) * depth_image[y, x] / self.fy
                    point_z = depth_image[y, x]

                    # Add normal vector
                    normal_x = normals[y, x, 0]
                    normal_y = normals[y, x, 1]
                    normal_z = normals[y, x, 2]

                    sampled_normals.extend([
                        [point_x, point_y, point_z],  # Point
                        [point_x + normal_x * 0.1, point_y + normal_y * 0.1, point_z + normal_z * 0.1]  # Normal endpoint
                    ])

        return np.array(sampled_normals)

    def publish_point_cloud(self, points, publisher, header):
        """
        Publish point cloud to ROS topic
        """
        if len(points) == 0:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_optical_frame"

        # Convert numpy array to PointCloud2
        from geometry_msgs.msg import Point32
        cloud_points = [Point32(x, y, z) for x, y, z in points]
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)

        publisher.publish(cloud_msg)

if __name__ == '__main__':
    processor = DepthCameraProcessor()
    rospy.spin()
```

### Performance Considerations for Depth Cameras

For humanoid robots with computational constraints:

- **Resolution Selection**: Balance detail vs. processing speed
- **ROI Processing**: Focus computation on relevant regions
- **Temporal Filtering**: Use previous frames to improve current estimates
- **Downsampling**: Reduce point cloud density where precision isn't critical
- **Multi-scale Processing**: Use different resolutions for different tasks

## IMU Simulation

Inertial Measurement Units (IMUs) are critical sensors for humanoid robots, providing information about orientation, angular velocity, and linear acceleration. This data is essential for balance control, motion detection, and navigation.

### IMU Physics and Characteristics

IMUs typically contain three types of sensors:

- **Gyroscopes**: Measure angular velocity (rate of rotation)
- **Accelerometers**: Measure linear acceleration (including gravity)
- **Magnetometers**: Measure magnetic field (for heading reference)

Key characteristics include:

- **Measurement Range**: Maximum measurable values (e.g., ±2000°/s for gyros)
- **Noise Density**: Noise per square root of bandwidth (e.g., 4 mdps/√Hz)
- **Bias Stability**: How much the bias drifts over time
- **Update Rate**: How frequently measurements are available (typically 100-1000 Hz)
- **Scale Factor Error**: Deviation from ideal measurement scale
- **Cross-Axis Sensitivity**: Response to inputs in non-sensitive axes

### Advanced IMU Configuration

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="humanoid_imu_advanced">
    <always_on>true</always_on>
    <update_rate>200</update_rate> <!-- Higher update rate for humanoid balance -->
    <visualize>false</visualize>
    <topic>__default_topic</topic>
    <imu>
      <!-- Angular velocity measurements -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.047e-3</stddev>  <!-- ~0.06 deg/s (low noise IMU) -->
            <bias_mean>0.0</bias_mean>
            <bias_stddev>8.727e-5</bias_stddev>  <!-- ~0.005 deg/s bias stability -->
            <dynamic_bias_std>8.727e-5</dynamic_bias_std>  <!-- Bias walk -->
            <dynamic_bias_correlation_time>100.0</dynamic_bias_correlation_time>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.047e-3</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>8.727e-5</bias_stddev>
            <dynamic_bias_std>8.727e-5</dynamic_bias_std>
            <dynamic_bias_correlation_time>100.0</dynamic_bias_correlation_time>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.047e-3</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>8.727e-5</bias_stddev>
            <dynamic_bias_std>8.727e-5</dynamic_bias_std>
            <dynamic_bias_correlation_time>100.0</dynamic_bias_correlation_time>
          </noise>
        </z>
      </angular_velocity>

      <!-- Linear acceleration measurements -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.0e-2</stddev>  <!-- 10 mg noise -->
            <bias_mean>0.0</bias_mean>
            <bias_stddev>1.0e-3</bias_stddev>  <!-- 1 mg bias -->
            <dynamic_bias_std>1.0e-4</dynamic_bias_std>  <!-- Bias walk -->
            <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.0e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>1.0e-3</bias_stddev>
            <dynamic_bias_std>1.0e-4</dynamic_bias_std>
            <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.0e-2</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>1.0e-3</bias_stddev>
            <dynamic_bias_std>1.0e-4</dynamic_bias_std>
            <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
          </noise>
        </z>
      </linear_acceleration>

      <!-- Angular velocity saturation limits -->
      <angular_velocity>
        <x>
          <saturation_level>3.49065850399</saturation_level> <!-- 200 deg/s -->
        </x>
        <y>
          <saturation_level>3.49065850399</saturation_level> <!-- 200 deg/s -->
        </y>
        <z>
          <saturation_level>3.49065850399</saturation_level> <!-- 200 deg/s -->
        </z>
      </angular_velocity>

      <!-- Linear acceleration saturation limits -->
      <linear_acceleration>
        <x>
          <saturation_level>196.2</saturation_level> <!-- 20g -->
        </x>
        <y>
          <saturation_level>196.2</saturation_level> <!-- 20g -->
        </y>
        <z>
          <saturation_level>196.2</saturation_level> <!-- 20g -->
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>/humanoid_robot/imu/data</topicName>
      <serviceName>/humanoid_robot/imu/service</serviceName>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <updateRate>200.0</updateRate>
      <gaussianNoise>1.0e-2</gaussianNoise>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
    </plugin>
  </sensor>
</gazebo>
```

### Multiple IMU Configuration for Humanoid Robots

Humanoid robots often have multiple IMUs for distributed sensing:

```xml
<!-- IMU in the head for orientation estimation -->
<gazebo reference="head_imu_link">
  <sensor type="imu" name="head_imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>1.222e-3</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1.222e-3</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1.222e-3</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
    <plugin name="head_imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>/humanoid_robot/head_imu</topicName>
      <frameName>head_imu_link</frameName>
      <updateRate>100.0</updateRate>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU in the torso for balance control -->
<gazebo reference="torso_imu_link">
  <sensor type="imu" name="torso_imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><stddev>8.73e-4</stddev></noise></x>
        <y><noise type="gaussian"><stddev>8.73e-4</stddev></noise></y>
        <z><noise type="gaussian"><stddev>8.73e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><stddev>1.0e-2</stddev></noise></x>
        <y><noise type="gaussian"><stddev>1.0e-2</stddev></noise></y>
        <z><noise type="gaussian"><stddev>1.0e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>
    <plugin name="torso_imu_plugin" filename="libgazebo_ros_imu.so">
      <topicName>/humanoid_robot/torso_imu</topicName>
      <frameName>torso_imu_link</frameName>
      <updateRate>200.0</updateRate>
    </plugin>
  </sensor>
</gazebo>
```

### Advanced IMU Data Processing

```python
#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from collections import deque

class AdvancedIMUProcessor:
    def __init__(self):
        rospy.init_node('advanced_imu_processor')

        # IMU subscribers (for multiple IMUs)
        self.head_imu_sub = rospy.Subscriber('/humanoid_robot/head_imu',
                                            Imu, self.head_imu_callback)
        self.torso_imu_sub = rospy.Subscriber('/humanoid_robot/torso_imu',
                                             Imu, self.torso_imu_callback)

        # Publishers
        self.orientation_pub = rospy.Publisher('/humanoid_robot/fused_orientation',
                                              Imu, queue_size=1)
        self.balance_pub = rospy.Publisher('/humanoid_robot/balance_state',
                                          Vector3, queue_size=1)

        # IMU state variables
        self.head_imu_data = None
        self.torso_imu_data = None
        self.last_head_time = None
        self.last_torso_time = None

        # Sensor fusion parameters
        self.gyro_bias = np.zeros(3)  # Gyro bias estimate
        self.orientation = R.from_quat([0, 0, 0, 1])  # Current orientation
        self.gravity = np.array([0, 0, -9.81])  # Gravity vector
        self.bias_stability = 1e-4  # Bias stability parameter

        # Complementary filter parameters
        self.acceleration_alpha = 0.01  # Low-pass filter coefficient for accelerometer
        self.magnetic_alpha = 0.01    # Low-pass filter coefficient for magnetometer

        # Data buffers for noise analysis
        self.gyro_buffer = deque(maxlen=1000)
        self.accel_buffer = deque(maxlen=1000)

        # Bias estimation parameters
        self.bias_learning_rate = 0.001
        self.stationary_threshold = 0.1  # Threshold for stationary detection

    def head_imu_callback(self, msg):
        """
        Process head IMU data
        """
        self.process_imu_data(msg, 'head')

    def torso_imu_callback(self, msg):
        """
        Process torso IMU data
        """
        self.process_imu_data(msg, 'torso')

    def process_imu_data(self, msg, sensor_name):
        """
        Process IMU data with advanced filtering
        """
        # Extract measurements
        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Update data buffers for noise analysis
        self.gyro_buffer.append(angular_velocity)
        self.accel_buffer.append(linear_acceleration)

        # Estimate bias if robot is likely stationary
        if self.is_stationary(angular_velocity, linear_acceleration):
            self.estimate_bias(angular_velocity)

        # Remove bias from angular velocity
        corrected_angular_velocity = angular_velocity - self.gyro_bias

        # Calculate time delta
        current_time = msg.header.stamp.to_sec()
        if hasattr(self, f'last_{sensor_name}_time') and getattr(self, f'last_{sensor_name}_time') is not None:
            dt = current_time - getattr(self, f'last_{sensor_name}_time')
        else:
            dt = 1.0/200.0  # Default to 200Hz if first measurement

        # Update time
        setattr(self, f'last_{sensor_name}_time', current_time)

        # Perform sensor fusion
        orientation_estimate = self.fuse_imu_data(
            corrected_angular_velocity,
            linear_acceleration,
            dt
        )

        # Publish processed data
        self.publish_processed_data(orientation_estimate, msg.header)

    def is_stationary(self, angular_velocity, linear_acceleration):
        """
        Determine if the robot is stationary based on IMU readings
        """
        # Check if angular velocity is low
        angular_magnitude = np.linalg.norm(angular_velocity)
        linear_magnitude = np.linalg.norm(linear_acceleration - self.gravity)

        return (angular_magnitude < self.stationary_threshold and
                linear_magnitude < self.stationary_threshold)

    def estimate_bias(self, angular_velocity):
        """
        Estimate and update gyro bias
        """
        # Simple bias estimation using exponential moving average
        self.gyro_bias = (1 - self.bias_learning_rate) * self.gyro_bias + \
                         self.bias_learning_rate * angular_velocity

    def fuse_imu_data(self, angular_velocity, linear_acceleration, dt):
        """
        Fuse gyroscope and accelerometer data using complementary filter
        """
        # Integrate gyroscope data for orientation
        rotation_vector = angular_velocity * dt
        delta_rotation = R.from_rotvec(rotation_vector)
        new_orientation = delta_rotation * self.orientation

        # Estimate orientation from accelerometer
        if np.linalg.norm(linear_acceleration) > 0.1:  # Avoid division by zero
            # Normalize accelerometer reading
            accel_normalized = linear_acceleration / np.linalg.norm(linear_acceleration)
            # Estimate gravity direction
            gravity_direction = -accel_normalized

            # Calculate roll and pitch from gravity vector
            pitch = np.arctan2(-gravity_direction[0],
                              np.sqrt(gravity_direction[1]**2 + gravity_direction[2]**2))
            roll = np.arctan2(gravity_direction[1], gravity_direction[2])

            # Create rotation from gravity (only roll and pitch)
            accel_orientation = R.from_euler('xyz', [roll, pitch, 0])

            # Apply complementary filter
            # Use accelerometer for low-frequency orientation (roll, pitch)
            # Use gyroscope for high-frequency updates
            current_euler = self.orientation.as_euler('xyz')
            accel_euler = accel_orientation.as_euler('xyz')

            # Complementary filter: blend accelerometer and gyroscope estimates
            filtered_roll = (1 - self.acceleration_alpha) * current_euler[0] + \
                           self.acceleration_alpha * accel_euler[0]
            filtered_pitch = (1 - self.acceleration_alpha) * current_euler[1] + \
                            self.acceleration_alpha * accel_euler[1]
            filtered_yaw = current_euler[2]  # Yaw from gyroscope integration

            gravity_based_orientation = R.from_euler('xyz', [filtered_roll, filtered_pitch, filtered_yaw])

            # Blend the two orientations
            self.orientation = R.slerp([self.orientation, new_orientation], [0.9, 0.1])[0]
            self.orientation = R.slerp([self.orientation, gravity_based_orientation], [0.7, 0.3])[0]
        else:
            # If accelerometer is unreliable, just use gyroscope
            self.orientation = new_orientation

        return self.orientation

    def publish_processed_data(self, orientation, header):
        """
        Publish processed IMU data
        """
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.header.frame_id = "base_link"

        # Set orientation
        quat = orientation.as_quat()
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # Set orientation covariance (diagonal of covariance matrix)
        # 0.01 rad = ~0.57 degrees uncertainty
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Calculate balance metrics
        euler = orientation.as_euler('xyz')
        balance_vector = Vector3()
        balance_vector.x = euler[0]  # Roll
        balance_vector.y = euler[1]  # Pitch
        balance_vector.z = euler[2]  # Yaw

        # Publish data
        self.orientation_pub.publish(imu_msg)
        self.balance_pub.publish(balance_vector)

    def analyze_noise_characteristics(self):
        """
        Analyze noise characteristics of IMU data
        """
        if len(self.gyro_buffer) < 100:
            return

        gyro_data = np.array(list(self.gyro_buffer))
        accel_data = np.array(list(self.accel_buffer))

        # Calculate statistics
        gyro_mean = np.mean(gyro_data, axis=0)
        gyro_std = np.std(gyro_data, axis=0)
        accel_mean = np.mean(accel_data, axis=0)
        accel_std = np.std(accel_data, axis=0)

        rospy.loginfo(f"IMU Noise Analysis:")
        rospy.loginfo(f"  Gyro - Mean: {gyro_mean}, Std: {gyro_std}")
        rospy.loginfo(f"  Accel - Mean: {accel_mean}, Std: {accel_std}")

        return {
            'gyro_mean': gyro_mean,
            'gyro_std': gyro_std,
            'accel_mean': accel_mean,
            'accel_std': accel_std
        }

if __name__ == '__main__':
    processor = AdvancedIMUProcessor()
    rate = rospy.Rate(10)  # 10Hz analysis

    while not rospy.is_shutdown():
        # Periodically analyze noise characteristics
        processor.analyze_noise_characteristics()
        rate.sleep()
```

### IMU Calibration Techniques

For accurate humanoid robot operation, IMU calibration is essential:

1. **Static Calibration**: Measure biases with the robot in known orientations
2. **Dynamic Calibration**: Characterize scale factors and cross-axis sensitivities
3. **Temperature Calibration**: Account for temperature-dependent drift
4. **Multi-position Calibration**: Calibrate across multiple orientations

```python
def perform_imu_calibration():
    """
    Example IMU calibration procedure
    """
    # Collect data in multiple static positions
    positions = [
        ("Level", [0, 0, 9.81]),      # Robot level, z-axis up
        ("Roll 90", [0, 9.81, 0]),    # Robot rolled 90 degrees
        ("Pitch 90", [9.81, 0, 0]),   # Robot pitched 90 degrees
        ("Upside down", [0, 0, -9.81]) # Robot upside down
    ]

    calibration_data = {}
    for name, expected_gravity in positions:
        print(f"Position {name}: Please place robot as indicated and press Enter")
        input()  # Wait for user to position robot

        # Collect data for calibration
        samples = []
        for _ in range(1000):  # Collect 1000 samples
            # In practice, read from IMU topic
            # samples.append(read_imu_acceleration())
            pass

        calibration_data[name] = {
            'samples': np.array(samples),
            'expected': np.array(expected_gravity)
        }

    # Calculate calibration parameters
    # This is a simplified example - real calibration is more complex
    return calibration_data
```

## Sensor Fusion Techniques

### Kalman Filter for Sensor Fusion

Combining multiple sensors for improved state estimation:

```python
import numpy as np

class SensorFusionKF:
    def __init__(self):
        # State: [x, y, z, vx, vy, vz, ax, ay, az]
        self.state_dim = 9
        self.obs_dim = 6  # Position + acceleration

        # Initialize state and covariance
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 1000  # High initial uncertainty

        # Process noise
        self.Q = np.eye(self.state_dim) * 0.1

        # Measurement noise
        self.R = np.eye(self.obs_dim) * 0.5

        # Observation matrix
        self.H = np.zeros((self.obs_dim, self.state_dim))
        # Position observations
        self.H[0:3, 0:3] = np.eye(3)  # Position
        self.H[3:6, 6:9] = np.eye(3)  # Acceleration

    def predict(self, dt):
        """
        Prediction step of Kalman filter
        """
        # State transition matrix (simplified model)
        F = np.eye(self.state_dim)
        F[0:3, 3:6] = dt * np.eye(3)  # Position from velocity
        F[3:6, 6:9] = dt * np.eye(3)  # Velocity from acceleration

        # Predict state
        self.x = F @ self.x

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        """
        Update step of Kalman filter
        """
        # Innovation
        y = measurement - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P

def sensor_fusion_example():
    """
    Example of fusing IMU and camera data
    """
    kf = SensorFusionKF()

    # Simulated measurements
    position = np.array([1.0, 2.0, 0.5])
    acceleration = np.array([0.1, -0.05, 0.02])
    measurement = np.concatenate([position, acceleration])

    # Time step
    dt = 0.01

    # Prediction and update
    kf.predict(dt)
    kf.update(measurement)

    return kf.x  # Fused state estimate
```

## Validation Methods

### Comparing Simulated vs. Real Data

Validating sensor simulation accuracy is crucial for trust in the simulation:

```python
import matplotlib.pyplot as plt
import numpy as np

def validate_sensor_simulation(simulated_data, real_data, sensor_type):
    """
    Validate simulated sensor data against real data
    """
    # Calculate statistical measures
    sim_mean = np.mean(simulated_data, axis=0)
    real_mean = np.mean(real_data, axis=0)

    sim_std = np.std(simulated_data, axis=0)
    real_std = np.std(real_data, axis=0)

    # Calculate correlation
    correlation = np.corrcoef(simulated_data.flatten(), real_data.flatten())[0, 1]

    # Calculate RMSE
    rmse = np.sqrt(np.mean((simulated_data - real_data) ** 2))

    print(f"Sensor Validation Results ({sensor_type}):")
    print(f"  Mean Difference: {np.abs(sim_mean - real_mean)}")
    print(f"  Std Dev Difference: {np.abs(sim_std - real_std)}")
    print(f"  Correlation: {correlation:.4f}")
    print(f"  RMSE: {rmse:.4f}")

    # Plot comparison
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 2, 1)
    plt.plot(simulated_data[:, 0], label='Simulated', alpha=0.7)
    plt.plot(real_data[:, 0], label='Real', alpha=0.7)
    plt.title(f'{sensor_type} - Axis 1 Comparison')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(simulated_data[:, 1], label='Simulated', alpha=0.7)
    plt.plot(real_data[:, 1], label='Real', alpha=0.7)
    plt.title(f'{sensor_type} - Axis 2 Comparison')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.scatter(real_data.flatten(), simulated_data.flatten(), alpha=0.5)
    plt.plot([real_data.min(), real_data.max()], [real_data.min(), real_data.max()], 'r--')
    plt.xlabel('Real Data')
    plt.ylabel('Simulated Data')
    plt.title('Scatter Plot (Perfect = Diagonal)')

    plt.subplot(2, 2, 4)
    plt.hist(simulated_data.flatten() - real_data.flatten(), bins=50, alpha=0.7, label='Simulated - Real')
    plt.title('Error Distribution')
    plt.legend()

    plt.tight_layout()
    plt.show()

    return {
        'correlation': correlation,
        'rmse': rmse,
        'mean_error': np.abs(sim_mean - real_mean),
        'std_error': np.abs(sim_std - real_std)
    }
```

## Exercises

1. **LiDAR Simulation**: Configure a LiDAR sensor in Gazebo and analyze the point cloud data it generates
2. **IMU Processing**: Implement an IMU bias estimation algorithm and test with simulated data
3. **Sensor Fusion**: Combine data from multiple simulated sensors using a Kalman filter

## Exercises

### Exercise 1: LiDAR Sensor Configuration
Configure and test a LiDAR sensor for humanoid robot navigation:
1. Set up a 2D LiDAR sensor with appropriate range and resolution for indoor navigation
2. Configure realistic noise models including Gaussian noise and bias
3. Implement a simple obstacle detection algorithm using the simulated LiDAR data
4. Test the sensor in various environments with different obstacle types
5. Analyze the impact of different noise parameters on detection performance

### Exercise 2: Depth Camera Integration
Integrate a depth camera into the humanoid robot simulation:
1. Configure RGB-D camera with appropriate intrinsics and distortion parameters
2. Implement object detection using both color and depth information
3. Create a point cloud processing pipeline for environment understanding
4. Test the system with various objects at different distances
5. Evaluate the accuracy of depth measurements compared to ground truth

### Exercise 3: IMU Calibration and Fusion
Calibrate and fuse IMU data for humanoid balance:
1. Configure multiple IMUs at different locations on the robot (head, torso, feet)
2. Implement bias estimation and correction algorithms
3. Develop a sensor fusion algorithm combining gyroscope and accelerometer data
4. Test the system during various robot motions (standing, walking, turning)
5. Validate the orientation estimates against ground truth data

### Exercise 4: Multi-Sensor Integration
Integrate multiple sensors for comprehensive environment perception:
1. Combine LiDAR, depth camera, and IMU data in a single perception pipeline
2. Implement sensor fusion techniques to improve perception accuracy
3. Create a unified representation of the environment using all sensor inputs
4. Test the integrated system in complex scenarios with dynamic obstacles
5. Evaluate the robustness of the multi-sensor system compared to single sensors

## Validation and Troubleshooting Tips

### Sensor Simulation Validation Techniques

Proper validation ensures that simulated sensors behave similarly to their real-world counterparts:

#### Cross-Validation with Real Sensors
- **Data Comparison**: Compare statistical properties of simulated vs. real sensor data
- **Performance Metrics**: Evaluate algorithms on both simulated and real data to ensure similar performance
- **Transfer Learning**: Test if models trained on simulated data perform well on real data

#### Common Validation Metrics
- **LiDAR**: Point cloud density, range accuracy, angular resolution
- **Depth Cameras**: Depth accuracy, FoV coverage, noise characteristics
- **IMUs**: Bias stability, noise spectral density, scale factor accuracy

#### Simulation-to-Reality Gap Assessment
```python
def assess_sim2real_gap(simulated_data, real_data, sensor_type):
    """
    Assess the gap between simulated and real sensor data
    """
    import numpy as np
    from scipy import stats

    # Calculate statistical measures
    sim_stats = {
        'mean': np.mean(simulated_data, axis=0),
        'std': np.std(simulated_data, axis=0),
        'min': np.min(simulated_data, axis=0),
        'max': np.max(simulated_data, axis=0)
    }

    real_stats = {
        'mean': np.mean(real_data, axis=0),
        'std': np.std(real_data, axis=0),
        'min': np.min(real_data, axis=0),
        'max': np.max(real_data, axis=0)
    }

    # Calculate similarity metrics
    mean_diff = np.abs(sim_stats['mean'] - real_stats['mean'])
    std_diff = np.abs(sim_stats['std'] - real_stats['std'])

    # Kolmogorov-Smirnov test for distribution similarity
    ks_statistic, p_value = stats.ks_2samp(
        simulated_data.flatten(),
        real_data.flatten()
    )

    print(f"Sensor Type: {sensor_type}")
    print(f"Mean Difference: {mean_diff}")
    print(f"Std Dev Difference: {std_diff}")
    print(f"KS Test Statistic: {ks_statistic:.4f}")
    print(f"KS Test P-Value: {p_value:.4f}")
    print(f"Distributions Similar: {p_value > 0.05}")

    return {
        'mean_diff': mean_diff,
        'std_diff': std_diff,
        'ks_statistic': ks_statistic,
        'p_value': p_value
    }
```

### Troubleshooting Common Issues

#### LiDAR Simulation Issues
- **Point Cloud Artifacts**: Check for incorrect noise parameters or range settings
- **Performance Problems**: Reduce sample count or update rate if simulation is slow
- **Inconsistent Data**: Verify sensor mounting position and orientation

#### Depth Camera Issues
- **Invalid Depth Values**: Check for proper clipping distances and noise modeling
- **Registration Problems**: Ensure RGB and depth images are properly synchronized
- **Noise Artifacts**: Adjust noise parameters to match real sensor characteristics

#### IMU Simulation Issues
- **Drift Accumulation**: Verify bias estimation and correction algorithms
- **Integration Errors**: Check time synchronization and update rates
- **Gravity Compensation**: Ensure proper accelerometer calibration

#### General Troubleshooting Tips
1. **Start Simple**: Begin with basic configurations before adding complexity
2. **Verify Parameters**: Double-check all sensor parameters against real hardware specs
3. **Monitor Performance**: Keep track of simulation update rates and computational load
4. **Validate Step-by-Step**: Test each sensor component individually before integration
5. **Document Assumptions**: Keep track of modeling assumptions and limitations

### Performance Optimization

#### Sensor-Specific Optimizations
- **LiDAR**: Reduce scan resolution when precision isn't critical
- **Cameras**: Use lower resolution or frame rate when possible
- **IMUs**: Use appropriate update rates (balance accuracy vs. performance)

#### Integration Optimizations
- **Data Filtering**: Pre-filter sensor data to reduce computational load
- **Parallel Processing**: Process multiple sensors in parallel when possible
- **Efficient Data Structures**: Use appropriate data structures for sensor data

## Summary

This chapter covered the simulation of various sensors used in humanoid robotics, including LiDAR, depth cameras, and IMUs. We explored configuration, noise modeling, data processing, and validation techniques. Proper sensor simulation is essential for developing robust perception algorithms that can transfer from simulation to reality.