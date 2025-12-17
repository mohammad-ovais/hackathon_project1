---
sidebar_position: 2
---

# Chapter 1: Introduction to Simulation Environments

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import CrossReference from '@site/src/components/CrossReference';
import Citation from '@site/src/components/Citation';
import Assessment from '@site/src/components/Assessment';

## Chapter Purpose

This chapter establishes foundational understanding of simulation environments and their role in the robotics development lifecycle. Students will learn about digital twin concepts, the comparison of different simulation platforms (Gazebo Classic, Gazebo Garden, Unity), and how to integrate simulation environments with ROS 2 systems. The chapter emphasizes the importance of simulation in safe testing, algorithm validation, and system optimization before deployment to physical hardware.

<LearningObjectives objectives={[
  "Understand digital twin concept and simulation lifecycle",
  "Compare Gazebo Classic, Gazebo Garden, and Unity for robotics applications",
  "Integrate simulation environments with ROS 2 communication patterns",
  "Set up basic simulation workflows from model creation to validation",
  "Validate simulation results against real-world performance expectations"
]} />

## Key Concepts

- **Digital twin concept and simulation lifecycle**: Understanding how simulation mirrors real-world systems
- **Comparison of Gazebo Classic, Gazebo Garden, and Unity**: Evaluating different simulation platforms for robotics
- **Integration with ROS 2 and communication patterns**: Connecting simulation to ROS 2 nodes and topics
- **Simulation workflow from model creation to validation**: Complete process from initial setup to validation

## Practical Demonstrations

### 1. Setting up Gazebo Classic and Unity Simulation Environments

Setting up Gazebo Classic for robotics simulation:

```bash
# Install Gazebo Classic
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev

# Launch basic Gazebo environment
gzserver --verbose

# In another terminal, launch GUI
gzclient --verbose
```

Setting up Unity Robotics Simulation Environment:

```bash
# Unity Robotics Simulation requires Unity Hub and specific packages
# 1. Install Unity Hub
# 2. Install Unity 2022.3 LTS
# 3. Import Unity Robotics Simulation Package
# 4. Set up ROS 2 TCP connection
```

### 2. Basic Robot Spawning and Control in Both Environments

Spawning a robot in Gazebo:

```xml
<!-- robot_model.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="chassis">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="chassis"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

### 3. Connecting Simulation to ROS 2 Workspace

Creating a Gazebo plugin to connect to ROS 2:

```cpp
// simple_robot_plugin.cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo {

class SimpleRobotPlugin : public ModelPlugin {
private:
  physics::ModelPtr model;
  physics::JointPtr leftWheelJoint;
  physics::JointPtr rightWheelJoint;
  event::ConnectionPtr updateConnection;

  // ROS 2 components
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the model pointer for convenience
    this->model = _parent;

    // Get joints
    this->leftWheelJoint = _parent->GetJoint("left_wheel_joint");
    this->rightWheelJoint = _parent->GetJoint("right_wheel_joint");

    // Initialize ROS 2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create ROS 2 node
    this->node = rclcpp::Node::make_shared("simple_robot_gazebo");

    // Create subscribers and publishers
    this->cmdVelSub = this->node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SimpleRobotPlugin::OnCmdVel, this, std::placeholders::_1));

    this->odomPub = this->node->create_publisher<nav_msgs::msg::Odometry>(
      "odom", 10);

    // Listen to the update event (Gazebo's simulated time)
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimpleRobotPlugin::OnUpdate, this));
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Apply wheel velocities based on twist command
    if (this->leftWheelJoint && this->rightWheelJoint) {
      this->leftWheelJoint->SetVelocity(0, msg->linear.x);
      this->rightWheelJoint->SetVelocity(0, msg->angular.z);
    }
  }

  void OnUpdate() {
    // Update odometry and publish
    // Implementation details...
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SimpleRobotPlugin)

}
```

## Hands-on Simulation Labs

<LabActivity title="Lab 2.1: Launching Basic Gazebo Simulation with TurtleBot3" time="30 min" difficulty="Easy">

### Objective
Launch a basic Gazebo simulation with a TurtleBot3 model and verify ROS 2 integration.

### Steps
1. Install TurtleBot3 packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations
   ```

2. Set simulation environment variables:
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models
   export TURTLEBOT3_MODEL=waffle
   ```

3. Launch TurtleBot3 in Gazebo:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

4. In another terminal, send movement commands:
   ```bash
   ros2 run turtlesim turtle_teleop_key
   # Or publish directly:
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}'
   ```

5. Verify that the robot moves in the simulation.

### Expected Outcome
TurtleBot3 model spawns in Gazebo simulation and responds to ROS 2 movement commands.

### Troubleshooting Tips
- Ensure all environment variables are set correctly
- Check that Gazebo server and client are running
- Verify that ROS 2 network is properly configured

</LabActivity>

<LabActivity title="Lab 2.2: Setting Up Unity Robotics Simulation Environment" time="45 min" difficulty="Medium">

### Objective
Set up Unity Robotics Simulation Environment and establish ROS 2 communication.

### Steps
1. Install Unity Hub and Unity 2022.3 LTS from unity.com

2. Download and import Unity Robotics Simulation Package:
   - Open Unity Hub
   - Create a new 3D project
   - Go to Window → Package Manager
   - Install "ROS TCP Connector" package
   - Install "Unity Perception" package (for synthetic data generation)

3. Set up ROS 2 TCP connection in Unity:
   ```csharp
   // In Unity, create a new C# script for ROS communication
   using Unity.Robotics.ROSTCPConnector;
   using UnityEngine;

   public class ROSConnectionHandler : MonoBehaviour
   {
       private ROSConnection ros;

       void Start()
       {
           // Get the ROS connection
           ros = ROSConnection.GetOrCreateInstance();

           // Set the IP address and port for ROS 2 communication
           ros.Initialize("127.0.0.1", 10000);  // Default ROS TCP Connector settings

           Debug.Log("ROS Connection Initialized");
       }

       void OnDestroy()
       {
           if (ros != null)
           {
               ros.Disconnect();
           }
       }
   }
   ```

4. Create a simple Unity scene with a robot model:
   - Add a basic robot model (or create simple geometric shapes)
   - Attach the ROS connection handler script
   - Create a simple controller to move the robot based on ROS messages

5. Test the connection by sending messages from ROS 2 to Unity:
   ```bash
   # In a terminal with ROS 2 sourced
   ros2 topic pub /unity_command std_msgs/msg/String "data: 'move_forward'"
   ```

### Expected Outcome
Unity environment connects to ROS 2 network and can receive/send messages.

### Troubleshooting Tips
- Ensure firewall allows communication on the specified port
- Check that Unity and ROS 2 are on the same network segment
- Verify that ROS TCP Connector is properly configured

</LabActivity>

<LabActivity title="Lab 2.3: Basic Robot Spawning and Control in Both Environments" time="60 min" difficulty="Hard">

### Objective
Create and control a custom robot model in both Gazebo and Unity simulation environments.

### Steps
1. Create a custom URDF model for your robot:
   ```xml
   <!-- custom_robot.urdf -->
   <?xml version="1.0"?>
   <robot name="custom_robot">
     <!-- Chassis link -->
     <link name="base_link">
       <visual>
         <geometry>
           <mesh filename="package://custom_robot/meshes/base.dae"/>
         </geometry>
         <origin xyz="0 0 0" rpy="0 0 0"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.4 0.3 0.15"/>
         </geometry>
         <origin xyz="0 0 0" rpy="0 0 0"/>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/>
       </inertial>
     </link>

     <!-- Sensor mount -->
     <joint name="lidar_mount_joint" type="fixed">
       <parent link="base_link"/>
       <child link="lidar_link"/>
       <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
     </joint>

     <link name="lidar_link">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.05"/>
         </geometry>
         <material name="gray">
           <color rgba="0.5 0.5 0.5 1.0"/>
         </material>
       </visual>
     </link>
   </robot>
   ```

2. Create a Gazebo world file:
   ```xml
   <!-- simple_room.world -->
   <?xml version="1.0" ?>
   <sdf version="1.6">
     <world name="simple_room">
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Simple room with walls -->
       <model name="wall_1">
         <pose>0 3 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>6 0.2 2</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>6 0.2 2</size>
               </box>
             </geometry>
             <material>
               <ambient>0.8 0.8 0.8 1</ambient>
               <diffuse>0.8 0.8 0.8 1</diffuse>
             </material>
           </visual>
         </link>
       </model>

       <!-- Spawn your robot -->
       <include>
         <uri>model://custom_robot</uri>
         <pose>0 0 0.1 0 0 0</pose>
       </include>
     </world>
   </sdf>
   ```

3. Create a launch file to spawn the robot in Gazebo:
   ```python
   # launch/custom_robot_simulation.launch.py
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       # Launch Gazebo with custom world
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               FindPackageShare("gazebo_ros"),
               "/launch/gazebo.launch.py"
           ]),
           launch_arguments={
               "world": PathJoinSubstitution([
                   FindPackageShare("custom_robot"),
                   "worlds",
                   "simple_room.world"
               ])
           }.items()
       )

       # Robot state publisher
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           parameters=[{'use_sim_time': True}]
       )

       # Spawn robot in Gazebo
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'custom_robot',
               '-x', '0', '-y', '0', '-z', '0.1'
           ],
           output='screen'
       )

       return LaunchDescription([
           gazebo,
           robot_state_publisher,
           spawn_entity
       ])
   ```

4. Test the simulation:
   ```bash
   # Build your package
   cd ~/ros2_ws
   colcon build --packages-select custom_robot
   source install/setup.bash

   # Launch the simulation
   ros2 launch custom_robot custom_robot_simulation.launch.py
   ```

5. For Unity, create equivalent robot model and scene with appropriate collision and visual properties.

### Expected Outcome
Custom robot model spawns in both Gazebo and Unity environments with proper physics properties.

### Troubleshooting Tips
- Ensure URDF is properly formatted and all referenced files exist
- Check that mesh files are in correct locations
- Verify that Gazebo models are properly installed

</LabActivity>

## Gazebo/Unity Tools & Plugins Used

- `gazebo_ros_pkgs`: Core ROS 2 plugins for Gazebo integration
- `Unity Robotics Simulation Package`: ROS 2 integration for Unity
- `RViz2` integration with simulation environments
- `ros2_control` for hardware interface simulation

## Physics, Sensors, and Environment Modeling

- **Basic physics properties and collision shapes**: Understanding mass, inertia, and collision geometry
- **Simple sensor integration**: Camera, laser, IMU in simulation
- **Basic environment setup**: Ground planes, lighting, obstacles

## Diagrams and Figures

### Simulation Architecture Diagram

```
+---------------------+
|   ROS 2 Nodes       |
| (Controllers, etc)  |
+---------------------+
         |
         v
+---------------------+
|  Simulation Engine  |
| (Gazebo/Unity)      |
+---------------------+
         |
         v
+---------------------+
|  Physics Engine     |
| (ODE, Bullet, etc)  |
+---------------------+
```

### Basic Simulation Workflow

```
Model Creation → Environment Setup → Physics Configuration → Sensor Integration → Validation → Deployment
```

### Comparison Chart of Simulation Platforms

| Feature | Gazebo Classic | Gazebo Garden | Unity |
|---------|----------------|---------------|-------|
| Physics | ODE, Bullet | More advanced physics engines | PhysX |
| Graphics | Basic OpenGL | Enhanced rendering | High-quality 3D |
| Sensors | Good selection | Expanded capabilities | Excellent for perception |
| Realism | Moderate | High | Very High |
| Learning Curve | Moderate | Moderate | Steeper |
| Use Cases | General robotics | Advanced simulation | Perception, VR/AR |

## Checklists

### ✓ Environment Setup Verification Checklist
- [ ] Simulation engine installed and running
- [ ] ROS 2 connection established
- [ ] Basic robot model spawns correctly
- [ ] Control commands received and executed

### ✓ Basic Simulation Launch Checklist
- [ ] World file properly configured
- [ ] Robot URDF correctly defined
- [ ] Spawn parameters verified
- [ ] Physics properties validated

### ✓ ROS 2 Connection Validation Checklist
- [ ] Topics properly advertised/subscribed
- [ ] Message types compatible
- [ ] Network configuration verified
- [ ] Communication frequency appropriate

## Glossary Terms

- **Digital Twin**: A virtual replica of a physical system that mirrors its real-world counterpart
- **Simulation Environment**: A virtual space where robotic systems can be tested and validated
- **Physics Engine**: Software that simulates physical interactions and dynamics
- **Collision Mesh**: Simplified geometry used for physics calculations
- **Sensor Simulation**: Virtual representation of real-world sensors
- **Scene Graph**: Hierarchical representation of objects in a 3D environment
- **Dynamic Objects**: Objects in simulation that can move and interact physically

## Cross-References

<CrossReference to="/docs/module-1/chapter-1" title="Chapter 1: Introduction to ROS 2 Architecture" type="prerequisite">
  Understanding ROS 2 architecture is essential for integrating simulation environments.
</CrossReference>

<CrossReference to="/docs/module-2/chapter-2" title="Chapter 2: Physics Modeling and Dynamics" type="continuation">
  Continue with advanced physics modeling after understanding basic simulation.
</CrossReference>

<CrossReference to="/docs/module-3/chapter-1" title="Module 3, Chapter 1: Introduction to NVIDIA Isaac Ecosystem" type="application">
  Apply simulation concepts in NVIDIA Isaac Sim for AI-robotics integration.
</CrossReference>

## Optional Advanced Section

### Custom Physics Engine Integration

Implementing custom physics simulation for specialized robotic applications.

### Cloud-based Simulation Services

Using cloud platforms like AWS RoboMaker or Azure Digital Twins for large-scale simulation.

<Assessment
  type="assignment"
  title="Chapter 1 Assessment: Introduction to Simulation Environments"
  objectives={[
    "Set up basic simulation environments in Gazebo and Unity",
    "Spawn and control custom robot models",
    "Establish ROS 2 communication with simulation",
    "Validate basic simulation functionality"
  ]}
  rubric={[
    {
      criterion: "Environment Setup",
      scores: [
        { label: "Excellent", value: "Both Gazebo and Unity environments properly configured with ROS 2 integration" },
        { label: "Proficient", value: "Environments configured with minor issues" },
        { label: "Developing", value: "One environment configured" },
        { label: "Beginning", value: "Environments not properly set up" }
      ]
    },
    {
      criterion: "Robot Model Integration",
      scores: [
        { label: "Excellent", value: "Custom robot model properly defined and spawns correctly" },
        { label: "Proficient", value: "Robot model functional with minor issues" },
        { label: "Developing", value: "Robot model partially implemented" },
        { label: "Beginning", value: "Robot model not properly implemented" }
      ]
    },
    {
      criterion: "ROS 2 Integration",
      scores: [
        { label: "Excellent", value: "Full ROS 2 communication with proper message handling" },
        { label: "Proficient", value: "ROS 2 communication functional with minor issues" },
        { label: "Developing", value: "ROS 2 communication partially implemented" },
        { label: "Beginning", value: "ROS 2 integration not properly implemented" }
      ]
    }
  ]}
>

### Assignment Tasks

1. **Environment Setup**: Install and configure both Gazebo Classic and Unity simulation environments with ROS 2 integration.

2. **Robot Model**: Create a custom URDF model of a simple robot with at least 2 links and 1 joint.

3. **Simulation Integration**: Create a launch file that spawns your robot in a Gazebo world and allows basic control.

4. **Validation**: Document the simulation setup process and validate basic functionality with movement commands.

### Submission Requirements

- URDF file for the custom robot model
- World file for the Gazebo environment
- Launch file for simulation setup
- Screenshots of the robot in simulation
- Documentation of the setup process
- Video demonstrating basic robot control in simulation

</Assessment>

## References

<Citation
  id="gazebo-ros-integration-2021"
  authors="Open Robotics"
  year="2021"
  title="Gazebo-ROS Integration for Robotics Simulation"
  source="ROS 2 Documentation"
  url="https://classic.gazebosim.org/tutorials?tut=ros2_overview"
>
  The official documentation on integrating Gazebo with ROS 2 for robotics simulation.
</Citation>

<Citation
  id="unity-robotics-simulation-2022"
  authors="Unity Technologies"
  year="2022"
  title="Unity Robotics Simulation Package"
  source="Unity Documentation"
  url="https://github.com/Unity-Technologies/Unity-Robotics-Hub"
>
  Documentation for Unity's robotics simulation tools and packages.
</Citation>