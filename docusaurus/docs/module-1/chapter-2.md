---
sidebar_position: 3
---

# Chapter 2: Nodes, Topics, and Messages

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import CrossReference from '@site/src/components/CrossReference';

## Chapter Purpose

This chapter covers the fundamental communication mechanism in ROS 2 using publishers and subscribers. Students will master the publisher-subscriber communication pattern, understand message types and serialization, and learn about Quality of Service (QoS) profiles and their impact on system performance and reliability.

<LearningObjectives objectives={[
  "Design and implement ROS 2 nodes in both Python and C++ for robot control and perception",
  "Configure and manage ROS 2 communication patterns including publisher-subscriber interactions",
  "Create and use custom message types for specialized data",
  "Configure appropriate QoS profiles for different communication requirements",
  "Debug and monitor topic-based communication in ROS 2 systems"
]} />

## Key Concepts

- **Node lifecycle and management**: Understanding the ROS 2 node lifecycle and proper resource management
- **Publisher-subscriber communication pattern**: The core asynchronous communication mechanism in ROS 2
- **Message types and serialization**: Understanding how data is structured and serialized for transmission
- **Quality of Service (QoS) profiles**: Configuring communication behavior for different reliability and performance requirements

## Practical Demonstrations

### 1. Creating Simple Publisher and Subscriber Nodes

Creating a publisher node that publishes sensor data:

```python
# publisher_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Creating a subscriber node that receives and processes messages:

```python
# subscriber_member_function.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Message Introspection and Debugging

Using ROS 2 command line tools to inspect messages:

```bash
# List all topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic std_msgs/msg/String

# Get information about a topic
ros2 topic info /topic

# Publish a message from command line
ros2 topic pub /topic std_msgs/String "data: 'Hello from CLI'"
```

### 3. QoS Profile Comparison and Selection

Understanding different QoS profiles:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Reliable communication with queue size of 10
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Best effort for real-time sensor data
sensor_qos = QoSProfile(
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT
)
```

## Hands-on Coding Labs

<LabActivity title="Lab 2.1: Implementing a Sensor Data Publisher (Simulated IMU)" time="45 min" difficulty="Medium">

### Objective
Create a ROS 2 publisher node that simulates IMU sensor data and publishes it to a topic.

### Steps
1. Create a new Python file for the IMU publisher:
   ```bash
   mkdir -p ~/ros2_ws/src/imu_publisher/imu_publisher
   touch ~/ros2_ws/src/imu_publisher/imu_publisher/imu_publisher_node.py
   ```

2. Add the following code to `imu_publisher_node.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu
   import math

   class ImuPublisher(Node):
       def __init__(self):
           super().__init__('imu_publisher')
           self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
           timer_period = 0.1  # 10Hz
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = Imu()
           # Simulate IMU data
           msg.orientation.x = 0.0
           msg.orientation.y = 0.0
           msg.orientation.z = math.sin(self.i * 0.1)
           msg.orientation.w = math.cos(self.i * 0.1)

           msg.angular_velocity.x = 0.1
           msg.angular_velocity.y = 0.05
           msg.angular_velocity.z = 0.02

           msg.linear_acceleration.x = 0.5
           msg.linear_acceleration.y = 0.2
           msg.linear_acceleration.z = 9.81  # gravity

           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing IMU data: {self.i}')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       imu_publisher = ImuPublisher()
       rclpy.spin(imu_publisher)
       imu_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Create a setup.py file for the package:
   ```python
   from setuptools import find_packages, setup

   package_name = 'imu_publisher'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='A simple IMU publisher for ROS 2',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'imu_publisher = imu_publisher.imu_publisher_node:main',
           ],
       },
   )
   ```

4. Create a package.xml file:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>imu_publisher</name>
     <version>0.0.0</version>
     <description>A simple IMU publisher for ROS 2</description>
     <maintainer email="your.email@example.com">Your Name</maintainer>
     <license>Apache-2.0</license>

     <depend>rclpy</depend>
     <depend>sensor_msgs</depend>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

5. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select imu_publisher
   source install/setup.bash
   ```

6. Run the publisher:
   ```bash
   ros2 run imu_publisher imu_publisher
   ```

### Expected Outcome
The IMU publisher node runs and publishes simulated IMU data to the `/imu/data` topic at 10Hz.

### Troubleshooting Tips
- Ensure all dependencies are properly declared in package.xml
- Check that the topic name matches between publisher and any subscribers

</LabActivity>

<LabActivity title="Lab 2.2: Creating a Subscriber to Visualize Sensor Data" time="45 min" difficulty="Medium">

### Objective
Create a ROS 2 subscriber node that receives IMU data and processes it to extract meaningful information.

### Steps
1. Create a new Python file for the IMU subscriber:
   ```bash
   touch ~/ros2_ws/src/imu_publisher/imu_publisher/imu_subscriber_node.py
   ```

2. Add the following code to `imu_subscriber_node.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Imu
   import math

   class ImuSubscriber(Node):
       def __init__(self):
           super().__init__('imu_subscriber')
           self.subscription = self.create_subscription(
               Imu,
               'imu/data',
               self.imu_callback,
               10)
           self.subscription  # prevent unused variable warning

       def imu_callback(self, msg):
           # Calculate yaw from quaternion
           quat = msg.orientation
           siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
           cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
           yaw = math.atan2(siny_cosp, cosy_cosp)

           self.get_logger().info(f'Yaw: {math.degrees(yaw):.2f}°, '
                                 f'Accel Z: {msg.linear_acceleration.z:.2f} m/s²')

   def main(args=None):
       rclpy.init(args=args)
       imu_subscriber = ImuSubscriber()
       rclpy.spin(imu_subscriber)
       imu_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Update the setup.py to include the new executable:
   ```python
   entry_points={
       'console_scripts': [
           'imu_publisher = imu_publisher.imu_publisher_node:main',
           'imu_subscriber = imu_publisher.imu_subscriber_node:main',
       ],
   },
   ```

4. Build and source the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select imu_publisher
   source install/setup.bash
   ```

5. In one terminal, run the publisher:
   ```bash
   ros2 run imu_publisher imu_publisher
   ```

6. In another terminal, run the subscriber:
   ```bash
   ros2 run imu_publisher imu_subscriber
   ```

### Expected Outcome
The subscriber node receives IMU data from the publisher and logs processed information including yaw angle and acceleration values.

### Troubleshooting Tips
- Ensure both nodes are using the same topic name
- Check that QoS profiles are compatible between publisher and subscriber

</LabActivity>

<LabActivity title="Lab 2.3: Implementing Custom Message Types for Specialized Data" time="60 min" difficulty="Hard">

### Objective
Create custom message types for specialized robot data and use them in publisher-subscriber communication.

### Steps
1. Create a new package for custom messages:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_cmake robot_msgs
   ```

2. Create msg directory and define a custom message:
   ```bash
   mkdir -p ~/ros2_ws/src/robot_msgs/msg
   ```

3. Create `RobotStatus.msg` file:
   ```bash
   # ~/ros2_ws/src/robot_msgs/msg/RobotStatus.msg
   uint8 STATE_IDLE=0
   uint8 STATE_MOVING=1
   uint8 STATE_ERROR=2

   uint8 state
   float64 battery_level
   string current_action
   float64[] joint_positions
   ```

4. Update CMakeLists.txt to include message generation:
   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/RobotStatus.msg"
   )

   ament_export_dependencies(rosidl_default_runtime)
   ```

5. Update package.xml to include message dependencies:
   ```xml
   <depend>std_msgs</depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

6. Build the message package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_msgs
   source install/setup.bash
   ```

7. Create a publisher using the custom message:
   ```python
   # ~/ros2_ws/src/robot_msgs/robot_msgs/robot_status_publisher.py
   import rclpy
   from rclpy.node import Node
   from robot_msgs.msg import RobotStatus

   class RobotStatusPublisher(Node):
       def __init__(self):
           super().__init__('robot_status_publisher')
           self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
           timer_period = 1.0  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = RobotStatus()
           msg.state = RobotStatus.STATE_MOVING if self.i % 3 != 0 else RobotStatus.STATE_IDLE
           msg.battery_level = 100.0 - (self.i * 0.1)
           msg.current_action = f'Moving to position {self.i}'
           msg.joint_positions = [float(i), float(i+1), float(i+2)]

           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: {msg.current_action}')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       robot_status_publisher = RobotStatusPublisher()
       rclpy.spin(robot_status_publisher)
       robot_status_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

8. Build and test the custom message:
   ```bash
   colcon build --packages-select robot_msgs
   source install/setup.bash
   ros2 run robot_msgs robot_status_publisher  # This would need to be set up properly
   ```

### Expected Outcome
Custom message types are defined, built, and can be used in ROS 2 publisher-subscriber communication.

### Troubleshooting Tips
- Ensure the message definition file has the correct syntax
- Check that all dependencies are properly declared in package.xml and CMakeLists.txt

</LabActivity>

## ROS 2 Packages/Tools Used

- `std_msgs`: Standard message types for basic data (String, Int, Float, etc.)
- `geometry_msgs`: Message types for geometry (Point, Pose, Vector3, etc.)
- `sensor_msgs`: Message types for sensors (Imu, LaserScan, Image, etc.)
- `rclpy` and `rclcpp` libraries: ROS 2 client libraries for Python and C++
- `ros2 topic` and `ros2 msg` tools: Command line tools for topic and message management

## Simulation vs Real-Robot Activities

### Simulation:
- Publishing simulated sensor data from Gazebo
- Testing communication patterns in virtual environments
- Validating message formats and QoS settings

### Real Robot:
- Connecting to actual IMU and publishing real sensor data
- Testing with actual hardware constraints and limitations
- Validating performance under real-world conditions

## Diagrams and Figures

### Publisher-Subscriber Communication Flow

```
+----------------+                    +------------------+
|   Publisher    |  ------> Topic ----->   Subscriber    |
|                |                    |                  |
| - Creates msg  |                    | - Receives msg   |
| - Publishes    |                    | - Processes msg  |
| - Manages node |                    | - Callback func  |
+----------------+                    +------------------+
```

### Message Type Hierarchy Diagram

```
Base Message Types
├── std_msgs/
│   ├── String, Int, Float, etc.
│   └── Common primitive types
├── geometry_msgs/
│   ├── Point, Pose, Vector3
│   └── Transformations
├── sensor_msgs/
│   ├── Imu, LaserScan, Image
│   └── Sensor data types
└── custom_msgs/
    └── Application-specific types
```

### QoS Profile Comparison Chart

| Profile | Reliability | Durability | History | Use Case |
|---------|-------------|------------|---------|----------|
| Sensor Data | Best Effort | Volatile | Keep Last | Real-time sensor streams |
| Command | Reliable | Volatile | Keep Last | Robot commands |
| Configuration | Reliable | Transient Local | Keep All | System parameters |
| Log | Reliable | Persistent | Keep All | Event logging |

## Checklists

### ✓ Node Creation and Lifecycle Validation Checklist
- [ ] Node properly initialized with correct name
- [ ] All resources properly created and configured
- [ ] Cleanup methods implemented for graceful shutdown
- [ ] Error handling implemented for edge cases

### ✓ Message Type Definition Checklist
- [ ] Appropriate standard message types used when available
- [ ] Custom message definitions properly structured
- [ ] Message dependencies properly declared
- [ ] Message validation implemented

### ✓ QoS Profile Selection Checklist
- [ ] QoS profile appropriate for data type and requirements
- [ ] Reliability setting matches use case (reliable vs best effort)
- [ ] Durability setting appropriate for publisher/subscriber relationship
- [ ] History policy and depth properly configured

## Glossary Terms

- **Publisher**: A ROS 2 node that sends messages on a topic
- **Subscriber**: A ROS 2 node that receives messages from a topic
- **Message**: The data structure exchanged between nodes
- **QoS (Quality of Service)**: Configuration parameters that define communication behavior
- **Callback**: Function executed when a message is received
- **Rate**: Frequency at which messages are published
- **Spin**: Process of checking for and processing messages
- **Topic**: Named bus over which messages are exchanged

## Cross-References

<CrossReference to="/docs/module-1/chapter-1" title="Chapter 1: Introduction to ROS 2 Architecture" type="prerequisite">
  Review the basic ROS 2 architecture concepts before diving into nodes and topics.
</CrossReference>

<CrossReference to="/docs/module-1/chapter-3" title="Chapter 3: Services and Actions" type="continuation">
  After mastering topics, continue with services and actions for synchronous communication.
</CrossReference>

<CrossReference to="/docs/module-2/chapter-1" title="Module 2, Chapter 1: Introduction to Simulation Environments" type="application">
  Apply node and topic concepts in simulation environments to test your implementations.
</CrossReference>

## Optional Advanced Section

### Custom Serialization and Deserialization

Implementing custom message serialization for performance-critical applications.

### Performance Optimization for High-Frequency Topics

Techniques for optimizing message publishing at high frequencies (>1kHz).

## Exercises

1. **Basic Publisher-Subscriber**: Create a simple publisher-subscriber pair that exchanges messages at 1Hz
2. **QoS Experimentation**: Compare different QoS profiles and observe their effects on message delivery
3. **Custom Message**: Define and use a custom message type for a specific robot application
4. **Performance Testing**: Measure and optimize the performance of high-frequency message publishing