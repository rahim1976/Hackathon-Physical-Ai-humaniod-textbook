# Nav2 Path Planning for Humanoid Robots

## Introduction

This chapter explores the application of Nav2 (Navigation 2) for bipedal humanoid path planning. Nav2 is the standard navigation framework for ROS 2, but requires specialized configuration and custom plugins to effectively handle the unique challenges of humanoid locomotion. You'll learn how to adapt Nav2 for humanoid robots, implement footstep planning, and ensure stable navigation with balance considerations.

The chapter will cover the fundamentals of humanoid navigation, custom Nav2 plugins for bipedal locomotion, and integration with Isaac Sim for realistic testing. We'll also explore advanced topics like dynamic obstacle avoidance and multi-terrain navigation specific to humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure Nav2 for bipedal humanoid navigation with balance considerations
- Implement custom plugins for humanoid-specific navigation requirements
- Integrate footstep planning with path planning for stable locomotion
- Handle dynamic obstacle avoidance for humanoid robots
- Optimize navigation performance for real-time humanoid operation

## Prerequisites

Before starting this chapter, you should have:
- Completed the Isaac Sim and Isaac ROS chapters (Module 3, Chapters 1-2)
- Understanding of ROS 2 navigation concepts
- Basic knowledge of humanoid kinematics and bipedal locomotion
- Familiarity with control theory concepts (ZMP, center of mass)

## Understanding Humanoid Navigation Challenges

### Differences from Wheeled Navigation

Humanoid navigation presents unique challenges compared to traditional wheeled robots:

1. **Balance Requirements**: Humanoid robots must maintain balance during navigation
2. **Footstep Planning**: Requires explicit footstep planning for stable locomotion
3. **Multi-terrain Navigation**: Must handle stairs, slopes, and uneven terrain
4. **Dynamic Stability**: Balance can be affected by external forces and obstacles
5. **Complex Kinematics**: 6+ DOF legs require sophisticated motion planning

### Key Considerations

- **Zero Moment Point (ZMP)**: Critical for maintaining balance during walking
- **Capture Point**: Determines where to place feet for stability
- **Center of Mass (CoM)**: Must be carefully controlled during navigation
- **Foot Placement**: Strategic foot placement for obstacle avoidance and stability

## Nav2 Architecture for Humanoid Robots

### Core Components

Nav2 consists of several key components that need adaptation for humanoid navigation:

1. **Global Planner**: Generates high-level path considering humanoid kinematics
2. **Local Planner**: Creates detailed footstep plans for immediate navigation
3. **Controller**: Executes footstep plans with balance control
4. **Recovery Behaviors**: Specialized recovery for humanoid-specific failures
5. **Lifecycle Manager**: Coordinates navigation state transitions

### Humanoid-Specific Modifications

```yaml
# nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    default_bt_xml_filename: "humanoid_navigator_bt.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_recover_nav_mesh_bt_node
    - nav2_follow_path_cancel_bt_node
```

## Custom Plugins for Humanoid Navigation

### Humanoid Global Planner

A custom global planner that considers humanoid-specific constraints:

```cpp
// Example: Humanoid Global Planner header
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.h"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.h"

namespace nav2_humanoid_planner
{

class HumanoidGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  HumanoidGlobalPlanner() = default;
  ~HumanoidGlobalPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Humanoid-specific parameters
  double step_length_max_;
  double step_width_max_;
  double min_step_clearance_;
  bool enable_footstep_planning_;

  // ZMP and balance constraints
  double zmp_margin_;
  double com_height_;

  // Costmap for humanoid navigation
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace nav2_humanoid_planner
```

### Footstep Planner Integration

Integrating footstep planning with Nav2 for stable humanoid navigation:

```python
# Example: Footstep planner integration
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Duration
import numpy as np

class FootstepPlanner:
    def __init__(self):
        self.step_length_max = 0.3  # meters
        self.step_width_max = 0.2   # meters
        self.step_rotation_max = 0.3  # radians
        self.com_height = 0.8       # meters (center of mass height)

    def plan_footsteps(self, path, robot_pose):
        """
        Convert navigation path to footstep plan for humanoid robot
        """
        footsteps = []

        # Convert path to footstep sequence
        for i in range(len(path.poses) - 1):
            start_pose = path.poses[i]
            end_pose = path.poses[i + 1]

            # Calculate required steps between poses
            step_sequence = self.calculate_step_sequence(start_pose, end_pose)
            footsteps.extend(step_sequence)

        return footsteps

    def calculate_step_sequence(self, start_pose, end_pose):
        """
        Calculate sequence of footsteps between two poses
        considering balance and kinematic constraints
        """
        # Calculate distance and direction
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Calculate number of steps needed
        num_steps = int(np.ceil(distance / self.step_length_max))

        # Generate footstep sequence
        footsteps = []
        for i in range(num_steps):
            ratio = (i + 1) / num_steps
            step_x = start_pose.pose.position.x + ratio * dx
            step_y = start_pose.pose.position.y + ratio * dy

            # Create footstep pose (left or right foot alternating)
            footstep_pose = PoseStamped()
            footstep_pose.header.frame_id = "map"
            footstep_pose.pose.position.x = step_x
            footstep_pose.pose.position.y = step_y
            footstep_pose.pose.position.z = 0.0  # Ground level

            # Set orientation
            footstep_pose.pose.orientation = start_pose.pose.orientation

            footsteps.append(footstep_pose)

        return footsteps

class HumanoidPathFollower(Node):
    def __init__(self):
        super().__init__('humanoid_path_follower')

        self.footstep_planner = FootstepPlanner()
        self.path_subscriber = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        self.footstep_publisher = self.create_publisher(
            Path, '/footstep_plan', 10
        )

    def path_callback(self, path_msg):
        # Get current robot pose
        robot_pose = self.get_current_pose()

        # Plan footsteps
        footsteps = self.footstep_planner.plan_footsteps(path_msg, robot_pose)

        # Publish footstep plan
        footstep_path = Path()
        footstep_path.header = path_msg.header
        footstep_path.poses = footsteps

        self.footstep_publisher.publish(footstep_path)
```

### Balance Controller Integration

Implementing balance control alongside navigation:

```python
# Example: Balance controller for humanoid navigation
class BalanceController:
    def __init__(self):
        self.com_height = 0.8  # Center of mass height
        self.zmp_reference = np.array([0.0, 0.0])  # Zero Moment Point reference
        self.com_reference = np.array([0.0, 0.0, self.com_height])

        # Control gains
        self.kp_com = np.array([10.0, 10.0, 0.0])  # Proportional gains for CoM
        self.kd_com = np.array([2.0, 2.0, 0.0])    # Derivative gains for CoM

    def compute_balance_control(self, current_com, current_com_vel, dt):
        """
        Compute balance control based on CoM position and velocity
        """
        # Calculate CoM error
        com_error = self.com_reference - current_com
        com_vel_error = -current_com_vel  # Assuming desired velocity is 0

        # Compute control forces
        control_force = self.kp_com * com_error + self.kd_com * com_vel_error

        return control_force

    def update_zmp_reference(self, foot_positions):
        """
        Update ZMP reference based on current foot positions
        """
        # Calculate support polygon from foot positions
        if len(foot_positions) == 2:  # Both feet on ground
            # Calculate center between feet
            zmp_ref = np.mean(foot_positions, axis=0)
        elif len(foot_positions) == 1:  # Single foot support
            # Use single foot position as reference
            zmp_ref = foot_positions[0]
        else:
            zmp_ref = np.array([0.0, 0.0])

        self.zmp_reference = zmp_ref
```

## Integration with Isaac Sim

### Simulation Setup for Humanoid Navigation

Configuring Isaac Sim for realistic humanoid navigation testing:

```python
# Example: Isaac Sim setup for humanoid navigation
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import carb

class HumanoidNavSimulation:
    def __init__(self):
        # Initialize simulation
        self.sim_app = SimulationApp({"headless": False})
        self.world = World(stage_units_in_meters=1.0)

        # Initialize ROS bridge
        from omni.isaac.ros_bridge import _ros_bridge
        self.ros_bridge = _ros_bridge.acquire_ros_bridge_interface()

    def setup_humanoid_navigation(self):
        """
        Set up humanoid robot with navigation capabilities in Isaac Sim
        """
        # Add humanoid robot to simulation
        asset_root = get_assets_root_path()
        if asset_root is None:
            carb.log_error("Could not find Isaac Sim assets. Please check your Isaac Sim installation.")
            return False

        # Load humanoid robot model
        humanoid_path = asset_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        # Note: Replace with actual humanoid robot model path
        add_reference_to_stage(usd_path=humanoid_path, prim_path="/World/Humanoid")

        # Set up navigation environment
        self.setup_navigation_environment()

        # Initialize the world
        self.world.reset()

        return True

    def setup_navigation_environment(self):
        """
        Set up navigation testing environment
        """
        # Add obstacles, ramps, stairs, etc.
        # Configure physics properties
        # Set up sensor configurations
        pass

    def run_navigation_simulation(self):
        """
        Run navigation simulation with Nav2 integration
        """
        # Main simulation loop
        while simulation_app.is_running():
            self.world.step(render=True)

            # Process ROS messages
            # Update navigation commands
            # Monitor robot state
            pass

# Usage example
def main():
    nav_sim = HumanoidNavSimulation()

    if nav_sim.setup_humanoid_navigation():
        nav_sim.run_navigation_simulation()
    else:
        print("Failed to set up navigation simulation")

    nav_sim.sim_app.close()
```

### Nav2 Configuration for Isaac Sim

Specific configuration for Nav2 when used with Isaac Sim:

```yaml
# Isaac Sim specific Nav2 parameters
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.3  # Adjust for humanoid robot
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.3  # Adjust for humanoid robot
      resolution: 0.1
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Dynamic Obstacle Avoidance for Humanoids

### Humanoid-Specific Collision Avoidance

Humanoid robots require special consideration for dynamic obstacle avoidance:

```python
# Example: Dynamic obstacle avoidance for humanoid robots
class HumanoidCollisionAvoidance:
    def __init__(self):
        self.robot_radius = 0.3  # Effective radius for humanoid
        self.step_time = 0.5     # Time for single step
        self.lookahead_time = 2.0  # Look ahead time for prediction

    def predict_collision(self, obstacle_pos, obstacle_vel, robot_pos, robot_vel):
        """
        Predict potential collision between humanoid and moving obstacle
        """
        # Calculate relative position and velocity
        rel_pos = obstacle_pos - robot_pos
        rel_vel = obstacle_vel - robot_vel

        # Calculate time to closest approach
        rel_speed_sq = np.dot(rel_vel, rel_vel)
        if rel_speed_sq < 1e-6:  # Obstacle and robot moving at same velocity
            dist = np.linalg.norm(rel_pos)
            return dist < (self.robot_radius + 0.2)  # Add safety margin

        t_ca = -np.dot(rel_pos, rel_vel) / rel_speed_sq
        t_ca = max(0, min(t_ca, self.lookahead_time))  # Clamp to lookahead time

        # Calculate closest approach distance
        closest_pos = rel_pos + t_ca * rel_vel
        closest_dist = np.linalg.norm(closest_pos)

        return closest_dist < (self.robot_radius + 0.2)  # Safety margin

    def compute_avoidance_velocity(self, current_vel, obstacles):
        """
        Compute avoidance velocity to prevent collisions
        """
        avoidance_force = np.array([0.0, 0.0])

        for obs in obstacles:
            obs_pos = np.array([obs.position.x, obs.position.y])
            obs_vel = np.array([obs.velocity.x, obs.velocity.y])
            robot_pos = np.array([current_vel.x, current_vel.y])  # Simplified

            # Calculate avoidance force based on distance and relative velocity
            to_robot = robot_pos - obs_pos
            dist = np.linalg.norm(to_robot)

            if dist < 2.0:  # Influence radius
                # Repulsive force
                force_mag = (2.0 - dist) / 2.0  # Decreases with distance
                force_dir = to_robot / dist if dist > 0.001 else np.array([1.0, 0.0])
                avoidance_force += force_mag * force_dir

        # Limit force magnitude
        force_norm = np.linalg.norm(avoidance_force)
        if force_norm > 1.0:
            avoidance_force = avoidance_force / force_norm

        return avoidance_force
```

### Multi-Terrain Navigation

Handling different terrain types for humanoid navigation:

```python
# Example: Multi-terrain navigation considerations
class MultiTerrainNavigator:
    def __init__(self):
        self.terrain_types = {
            'flat': {'max_slope': 0.1, 'step_height': 0.05, 'traction': 1.0},
            'grass': {'max_slope': 0.2, 'step_height': 0.05, 'traction': 0.8},
            'stairs': {'max_slope': 0.8, 'step_height': 0.18, 'traction': 0.9},
            'ramp': {'max_slope': 0.3, 'step_height': 0.05, 'traction': 0.7},
            'rough': {'max_slope': 0.15, 'step_height': 0.08, 'traction': 0.6}
        }

    def evaluate_terrain_traversability(self, terrain_map, robot_pose):
        """
        Evaluate terrain traversability for humanoid robot
        """
        traversability_scores = {}

        for terrain_type, properties in self.terrain_types.items():
            # Calculate traversability based on terrain properties
            slope_penalty = min(1.0, abs(properties['max_slope']) / 0.5)
            step_penalty = min(1.0, properties['step_height'] / 0.2)
            traction_score = properties['traction']

            # Combine penalties into traversability score
            traversability = traction_score * (1 - slope_penalty) * (1 - step_penalty)
            traversability_scores[terrain_type] = traversability

        return traversability_scores

    def adjust_navigation_for_terrain(self, current_terrain, base_velocity):
        """
        Adjust navigation parameters based on current terrain
        """
        terrain_props = self.terrain_types.get(current_terrain, self.terrain_types['flat'])

        # Adjust velocity based on terrain
        adjusted_velocity = base_velocity * terrain_props['traction']

        # Adjust step parameters
        step_params = {
            'max_step_length': min(0.3, 0.3 * terrain_props['traction']),
            'max_step_width': min(0.2, 0.2 * terrain_props['traction']),
            'step_height_tolerance': terrain_props['step_height']
        }

        return adjusted_velocity, step_params
```

## Performance Optimization

### Real-time Navigation Considerations

Optimizing Nav2 for real-time humanoid navigation:

```bash
# Example: Optimized launch parameters for real-time operation
# launch_realtime_nav2.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Real-time optimized parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_params = os.path.join(
        get_package_share_directory('nav2_humanoid_bringup'),
        'params',
        'nav2_realtime_params.yaml'
    )

    # Navigation nodes with real-time priority
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoverer_server',
                       'bt_navigator',
                       'waypoint_follower']

    return LaunchDescription([
        # Real-time optimized controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--priority', '80']  # Real-time priority
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        )
    ])
```

### Memory and Computation Optimization

Efficient resource usage for humanoid navigation:

- **Path Caching**: Cache computed paths for similar destinations
- **Incremental Updates**: Update maps incrementally rather than full recalculation
- **Multi-resolution Maps**: Use different map resolutions for different planning needs
- **Predictive Planning**: Pre-compute likely navigation paths

## Practical Examples

### Example 1: Simple Humanoid Navigation

```python
# Example: Simple humanoid navigation with Nav2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait for Nav2 to be available
        self.nav_client.wait_for_server()

    def navigate_to_pose(self, x, y, theta):
        """
        Navigate humanoid robot to specified pose
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set target position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        import math
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Send navigation goal
        self.get_logger().info(f'Navigating to position: ({x}, {y}, {theta})')

        future = self.nav_client.send_goal_async(goal_msg)
        return future

def main(args=None):
    rclpy.init(args=args)

    navigator = HumanoidNavigator()

    # Example navigation to a specific location
    future = navigator.navigate_to_pose(5.0, 3.0, 0.0)

    # Wait for completion
    rclpy.spin_until_future_complete(navigator, future)

    result = future.result()
    if result:
        navigator.get_logger().info('Navigation completed successfully')
    else:
        navigator.get_logger().error('Navigation failed')

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Complex Navigation with Obstacle Avoidance

Implementing navigation with dynamic obstacle avoidance and multi-terrain handling.

## Troubleshooting and Validation

### Common Navigation Issues

1. **Path Planning Failures**
   - Check costmap configuration
   - Verify robot footprint settings
   - Ensure proper map resolution

2. **Balance Issues During Navigation**
   - Adjust step timing parameters
   - Verify CoM height estimation
   - Check footstep planning accuracy

3. **Dynamic Obstacle Collisions**
   - Increase safety margins
   - Improve obstacle detection
   - Adjust prediction horizons

### Validation Methods

```python
# Example: Navigation validation functions
def validate_navigation_performance(nav_results):
    """
    Validate navigation performance metrics
    """
    metrics = {
        'success_rate': 0.0,
        'avg_time': 0.0,
        'avg_path_efficiency': 0.0,
        'safety_compliance': 0.0
    }

    successful_navigations = [r for r in nav_results if r.success]
    if successful_navigations:
        metrics['success_rate'] = len(successful_navigations) / len(nav_results)
        metrics['avg_time'] = sum(r.time for r in successful_navigations) / len(successful_navigations)

        # Calculate path efficiency (optimal path length / actual path length)
        path_efficiencies = [r.optimal_length / r.actual_length for r in successful_navigations if r.actual_length > 0]
        if path_efficiencies:
            metrics['avg_path_efficiency'] = sum(path_efficiencies) / len(path_efficiencies)

    # Safety compliance (percentage of navigations without collisions)
    safe_navigations = [r for r in nav_results if not r.collision]
    if nav_results:
        metrics['safety_compliance'] = len(safe_navigations) / len(nav_results)

    return metrics
```

## Exercises

1. **Nav2 Configuration**: Configure Nav2 for a specific humanoid robot model with appropriate parameters
2. **Footstep Planning**: Implement a basic footstep planner that works with Nav2's path planning
3. **Simulation Testing**: Test navigation in Isaac Sim with various obstacle configurations
4. **Performance Analysis**: Compare navigation performance with and without humanoid-specific modifications

## Summary

This chapter has covered Nav2 path planning specifically adapted for humanoid robots, including custom plugins, footstep planning integration, and balance considerations. You've learned how to configure Nav2 for the unique challenges of bipedal locomotion and integrate it with Isaac Sim for realistic testing.

## Next Steps

The Isaac AI Brain module is now complete! You have learned about Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated VSLAM, and Nav2 for humanoid path planning. These technologies together form a powerful platform for developing advanced humanoid robotics applications.