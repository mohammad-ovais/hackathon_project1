---
sidebar_position: 6
---

# Chapter 5: TF and Navigation Fundamentals

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import CrossReference from '@site/src/components/CrossReference';
import Citation from '@site/src/components/Citation';
import Assessment from '@site/src/components/Assessment';

## Chapter Purpose

This chapter covers coordinate frame management and spatial relationships essential for robot navigation. Students will learn to create TF broadcasters for robot links, visualize TF trees with rviz2, perform transform lookups between coordinate frames, and understand how TF integrates with navigation systems. The chapter emphasizes practical application of TF in robotic perception and motion planning.

<LearningObjectives objectives={[
  "Create TF broadcasters for robot state and coordinate frame management",
  "Visualize TF trees with rviz2 for debugging and analysis",
  "Perform transform lookups and point transformations between frames",
  "Integrate TF with navigation stack for spatial reasoning",
  "Build URDF models with proper TF frame definitions"
]} />

## Key Concepts

- **Transform (TF) tree and coordinate frame management**: Understanding the hierarchical structure of coordinate frames
- **Static and dynamic transforms**: Distinguishing between fixed and changing coordinate relationships
- **Coordinate frame conversions and transformations**: Converting points between different coordinate systems
- **Basic navigation stack integration**: Using TF for spatial reasoning in navigation systems

## Practical Demonstrations

### 1. Creating TF Broadcasters for Robot Links

Creating a broadcaster for a simple robot with multiple links:

```python
# tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Define static transforms for robot
        self.base_to_laser = TransformStamped()
        self.base_to_laser.header.frame_id = 'base_link'
        self.base_to_laser.child_frame_id = 'laser_frame'
        self.base_to_laser.transform.translation.x = 0.1  # 10cm forward
        self.base_to_laser.transform.translation.y = 0.0
        self.base_to_laser.transform.translation.z = 0.2  # 20cm up
        # No rotation - identity quaternion
        self.base_to_laser.transform.rotation.x = 0.0
        self.base_to_laser.transform.rotation.y = 0.0
        self.base_to_laser.transform.rotation.z = 0.0
        self.base_to_laser.transform.rotation.w = 1.0

    def publish_static_transforms(self):
        # Publish static transforms
        self.base_to_laser.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.base_to_laser)

class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)  # 10 Hz
        self.theta = 0.0

    def broadcast_tf(self):
        t = TransformStamped()

        # Define coordinate frames
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rotating_frame'

        # Set translation
        t.transform.translation.x = 0.5 * math.cos(self.theta)
        t.transform.translation.y = 0.5 * math.sin(self.theta)
        t.transform.translation.z = 0.0

        # Set rotation (rotate around Z axis)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)
        self.theta += 0.1

def main(args=None):
    rclpy.init(args=args)

    # Create both static and dynamic publishers
    static_publisher = StaticFramePublisher()
    dynamic_publisher = DynamicFramePublisher()

    # Publish static transforms once
    static_publisher.publish_static_transforms()

    try:
        # Run the dynamic publisher
        rclpy.spin(dynamic_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        dynamic_publisher.destroy_node()
        static_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Visualizing TF Tree with rviz2

Setting up rviz2 to visualize the TF tree:

```bash
# Launch rviz2 and add TF display
ros2 run rviz2 rviz2

# In rviz2:
# 1. Add by topic -> TF
# 2. Set Fixed Frame to 'base_link' or 'map'
# 3. Adjust visualization settings (axes size, etc.)
```

### 3. Transform Lookup and Point Transformations

Performing transform lookups between frames:

```python
# tf_lookup_client.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point

class TFClient(Node):
    def __init__(self):
        super().__init__('tf_client')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

        # Create a publisher for transformed points
        self.point_pub = self.create_publisher(PointStamped, 'transformed_point', 10)

    def lookup_transform(self):
        try:
            # Look up transform from 'laser_frame' to 'base_link'
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'laser_frame',
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'Transform from laser_frame to base_link: '
                f'x={trans.transform.translation.x:.3f}, '
                f'y={trans.transform.translation.y:.3f}, '
                f'z={trans.transform.translation.z:.3f}'
            )

            # Example: Transform a point from laser frame to base frame
            point_in_laser = PointStamped()
            point_in_laser.header.frame_id = 'laser_frame'
            point_in_laser.header.stamp = self.get_clock().now().to_msg()
            point_in_laser.point.x = 1.0  # 1m in front of laser
            point_in_laser.point.y = 0.0
            point_in_laser.point.z = 0.0

            # Transform the point
            point_in_base = do_transform_point(point_in_laser, trans)
            self.get_logger().info(
                f'Point in base_link: x={point_in_base.point.x:.3f}, '
                f'y={point_in_base.point.y:.3f}, z={point_in_base.point.z:.3f}'
            )

            # Publish the transformed point
            point_in_base.header.frame_id = 'base_link'
            point_in_base.header.stamp = self.get_clock().now().to_msg()
            self.point_pub.publish(point_in_base)

        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    tf_client = TFClient()

    try:
        rclpy.spin(tf_client)
    except KeyboardInterrupt:
        pass
    finally:
        tf_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Coding Labs

<LabActivity title="Lab 5.1: Building a URDF Model with TF Frames" time="60 min" difficulty="Hard">

### Objective
Create a URDF (Unified Robot Description Format) model with proper TF frame definitions for a robot.

### Steps
1. Create a URDF file for a simple differential drive robot:
   ```xml
   <!-- ~/ros2_ws/src/robot_description/urdf/robot.urdf -->
   <?xml version="1.0"?>
   <robot name="differential_drive_robot">
     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.5 0.3 0.15"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.5 0.3 0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Left Wheel -->
     <joint name="left_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
     </joint>
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Right Wheel -->
     <joint name="right_wheel_joint" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
     </joint>
     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
         <material name="black">
           <color rgba="0 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Laser Sensor -->
     <joint name="laser_joint" type="fixed">
       <parent link="base_link"/>
       <child link="laser_frame"/>
       <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
     </joint>
     <link name="laser_frame">
       <visual>
         <geometry>
           <box size="0.05 0.05 0.05"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 0.8"/>
         </material>
       </visual>
     </link>

     <!-- IMU Sensor -->
     <joint name="imu_joint" type="fixed">
       <parent link="base_link"/>
       <child link="imu_frame"/>
       <origin xyz="0 0 0.05" rpy="0 0 0"/>
     </joint>
     <link name="imu_frame"/>
   </robot>
   ```

2. Create a launch file to publish the robot state:
   ```python
   # ~/ros2_ws/src/robot_description/launch/robot_state.launch.py
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   import os

   def generate_launch_description():
       # Declare launch arguments
       use_sim_time = LaunchConfiguration('use_sim_time')

       # Get URDF path
       urdf_path = os.path.join(
           os.path.dirname(__file__),
           '..',
           'urdf',
           'robot.urdf'
       )

       # Read URDF file
       with open(urdf_path, 'r') as infp:
           robot_desc = infp.read()

       # Robot State Publisher node
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           parameters=[
               {'use_sim_time': use_sim_time},
               {'robot_description': robot_desc}
           ],
           output='screen'
       )

       # Joint State Publisher (for static joints in this case)
       joint_state_publisher = Node(
           package='joint_state_publisher',
           executable='joint_state_publisher',
           name='joint_state_publisher',
           parameters=[
               {'use_sim_time': use_sim_time}
           ],
           output='screen'
       )

       return LaunchDescription([
           robot_state_publisher,
           joint_state_publisher
       ])
   ```

3. Create a Python script to visualize the robot in RViz:
   ```python
   # ~/ros2_ws/src/robot_description/robot_description/rviz_spawner.py
   import rclpy
   from rclpy.node import Node
   from visualization_msgs.msg import Marker
   import math

   class RvizSpawner(Node):
       def __init__(self):
           super().__init__('rviz_spawner')
           self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
           self.timer = self.create_timer(1.0, self.publish_markers)

       def publish_markers(self):
           # Create a simple marker for visualization
           marker = Marker()
           marker.header.frame_id = "base_link"
           marker.header.stamp = self.get_clock().now().to_msg()
           marker.ns = "robot_description"
           marker.id = 0
           marker.type = Marker.CUBE
           marker.action = Marker.ADD

           # Set position and size
           marker.pose.position.x = 0.0
           marker.pose.position.y = 0.0
           marker.pose.position.z = 0.0
           marker.pose.orientation.x = 0.0
           marker.pose.orientation.y = 0.0
           marker.pose.orientation.z = 0.0
           marker.pose.orientation.w = 1.0
           marker.scale.x = 0.5
           marker.scale.y = 0.3
           marker.scale.z = 0.15
           marker.color.a = 0.8  # Don't forget to set the alpha!
           marker.color.r = 0.0
           marker.color.g = 0.0
           marker.color.b = 1.0

           self.marker_publisher.publish(marker)

   def main(args=None):
       rclpy.init(args=args)
       rviz_spawner = RvizSpawner()

       try:
           rclpy.spin(rviz_spawner)
       except KeyboardInterrupt:
           pass
       finally:
           rviz_spawner.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Build and test the URDF model:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_description
   source install/setup.bash

   # Launch the robot state publisher
   ros2 launch robot_description robot_state.launch.py
   ```

5. Visualize in RViz:
   ```bash
   # In another terminal
   ros2 run rviz2 rviz2
   # Add RobotModel display and set Robot Description to /robot_description
   # Add TF display to see the transform tree
   ```

### Expected Outcome
A properly defined URDF model with coordinate frames that can be visualized in RViz and used by the TF system.

### Troubleshooting Tips
- Check that all joint names and link names match between URDF and TF
- Ensure URDF file is properly formatted XML
- Verify that robot_state_publisher is running to publish static transforms

</LabActivity>

<LabActivity title="Lab 5.2: Implementing TF Broadcasters for Robot State" time="60 min" difficulty="Hard">

### Objective
Create TF broadcasters that publish dynamic transforms based on robot state (odometry, joint states).

### Steps
1. Create a TF broadcaster that publishes robot odometry transforms:
   ```python
   # ~/ros2_ws/src/robot_tf/robot_tf/odometry_broadcaster.py
   import rclpy
   from rclpy.node import Node
   from nav_msgs.msg import Odometry
   from geometry_msgs.msg import TransformStamped
   from tf2_ros import TransformBroadcaster
   import math

   class OdometryBroadcaster(Node):
       def __init__(self):
           super().__init__('odometry_broadcaster')

           # Initialize robot pose
           self.x = 0.0
           self.y = 0.0
           self.theta = 0.0

           # Create TF broadcaster
           self.tf_broadcaster = TransformBroadcaster(self)

           # Subscribe to odometry topic
           self.subscription = self.create_subscription(
               Odometry,
               'odom',
               self.odom_callback,
               10
           )
           self.subscription  # prevent unused variable warning

       def odom_callback(self, msg):
           # Extract pose from odometry message
           self.x = msg.pose.pose.position.x
           self.y = msg.pose.pose.position.y

           # Convert quaternion to yaw (simplified for 2D)
           q = msg.pose.pose.orientation
           siny_cosp = 2 * (q.w * q.z + q.x * q.y)
           cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
           self.theta = math.atan2(siny_cosp, cosy_cosp)

           # Publish transform from odom to base_link
           t = TransformStamped()
           t.header.stamp = self.get_clock().now().to_msg()
           t.header.frame_id = 'odom'
           t.child_frame_id = 'base_link'

           t.transform.translation.x = self.x
           t.transform.translation.y = self.y
           t.transform.translation.z = 0.0

           t.transform.rotation.x = 0.0
           t.transform.rotation.y = 0.0
           t.transform.rotation.z = math.sin(self.theta / 2.0)
           t.transform.rotation.w = math.cos(self.theta / 2.0)

           self.tf_broadcaster.sendTransform(t)

   def main(args=None):
       rclpy.init(args=args)
       odometry_broadcaster = OdometryBroadcaster()

       try:
           rclpy.spin(odometry_broadcaster)
       except KeyboardInterrupt:
           pass
       finally:
           odometry_broadcaster.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create a joint state broadcaster that publishes dynamic joint transforms:
   ```python
   # ~/ros2_ws/src/robot_tf/robot_tf/joint_state_broadcaster.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import TransformStamped
   from tf2_ros import TransformBroadcaster
   import math

   class JointStateBroadcaster(Node):
       def __init__(self):
           super().__init__('joint_state_broadcaster')

           # Initialize joint positions
           self.left_wheel_pos = 0.0
           self.right_wheel_pos = 0.0

           # Create TF broadcaster
           self.tf_broadcaster = TransformBroadcaster(self)

           # Subscribe to joint states
           self.subscription = self.create_subscription(
               JointState,
               'joint_states',
               self.joint_state_callback,
               10
           )
           self.subscription  # prevent unused variable warning

       def joint_state_callback(self, msg):
           # Update joint positions
           for i, name in enumerate(msg.name):
               if name == 'left_wheel_joint':
                   self.left_wheel_pos = msg.position[i]
               elif name == 'right_wheel_joint':
                   self.right_wheel_pos = msg.position[i]

           # Publish transform for left wheel
           left_t = TransformStamped()
           left_t.header.stamp = self.get_clock().now().to_msg()
           left_t.header.frame_id = 'base_link'
           left_t.child_frame_id = 'left_wheel'

           left_t.transform.translation.x = 0.0
           left_t.transform.translation.y = 0.15  # offset from base
           left_t.transform.translation.z = -0.05  # slightly below base

           # Rotate wheel based on position
           left_t.transform.rotation.x = math.sin(self.left_wheel_pos / 2.0)
           left_t.transform.rotation.y = 0.0
           left_t.transform.rotation.z = 0.0
           left_t.transform.rotation.w = math.cos(self.left_wheel_pos / 2.0)

           self.tf_broadcaster.sendTransform(left_t)

           # Publish transform for right wheel
           right_t = TransformStamped()
           right_t.header.stamp = self.get_clock().now().to_msg()
           right_t.header.frame_id = 'base_link'
           right_t.child_frame_id = 'right_wheel'

           right_t.transform.translation.x = 0.0
           right_t.transform.translation.y = -0.15  # offset from base
           right_t.transform.translation.z = -0.05  # slightly below base

           # Rotate wheel based on position
           right_t.transform.rotation.x = math.sin(self.right_wheel_pos / 2.0)
           right_t.transform.rotation.y = 0.0
           right_t.transform.rotation.z = 0.0
           right_t.transform.rotation.w = math.cos(self.right_wheel_pos / 2.0)

           self.tf_broadcaster.sendTransform(right_t)

   def main(args=None):
       rclpy.init(args=args)
       joint_state_broadcaster = JointStateBroadcaster()

       try:
           rclpy.spin(joint_state_broadcaster)
       except KeyboardInterrupt:
           pass
       finally:
           joint_state_broadcaster.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Create a launch file to run both broadcasters:
   ```python
   # ~/ros2_ws/src/robot_tf/launch/tf_broadcasters.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='robot_tf',
               executable='odometry_broadcaster',
               name='odometry_broadcaster',
               output='screen'
           ),
           Node(
               package='robot_tf',
               executable='joint_state_broadcaster',
               name='joint_state_broadcaster',
               output='screen'
           )
       ])
   ```

4. Test the TF broadcasters:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_tf
   source install/setup.bash

   # Run the broadcasters
   ros2 launch robot_tf tf_broadcasters.launch.py
   ```

5. Test TF transforms:
   ```bash
   # In another terminal, check available transforms
   ros2 run tf2_tools view_frames

   # Look up a specific transform
   ros2 run tf2_ros tf2_echo odom base_link
   ```

### Expected Outcome
TF broadcasters that publish dynamic transforms based on robot state, creating a complete transform tree that updates in real-time.

### Troubleshooting Tips
- Ensure timestamp in transforms is current
- Check that frame IDs match between publishers and subscribers
- Verify that the TF tree is connected from base to all frames

</LabActivity>

<LabActivity title="Lab 5.3: Transforming Coordinates Between Frames" time="45 min" difficulty="Medium">

### Objective
Implement coordinate frame transformations to convert points between different coordinate systems.

### Steps
1. Create a transform listener and converter:
   ```python
   # ~/ros2_ws/src/robot_tf/robot_tf/transform_converter.py
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PointStamped, Point
   from tf2_ros import TransformListener, Buffer
   from tf2_geometry_msgs import do_transform_point
   import math

   class TransformConverter(Node):
       def __init__(self):
           super().__init__('transform_converter')

           # Create TF buffer and listener
           self.tf_buffer = Buffer()
           self.tf_listener = TransformListener(self.tf_buffer, self)

           # Create publisher for transformed points
           self.pub = self.create_publisher(PointStamped, 'transformed_points', 10)

           # Create subscription for points to transform
           self.sub = self.create_subscription(
               PointStamped,
               'input_points',
               self.transform_point,
               10
           )

           # Timer to periodically transform a sample point
           self.timer = self.create_timer(2.0, self.transform_sample_point)

       def transform_point(self, point_msg):
           try:
               # Transform the incoming point
               transform = self.tf_buffer.lookup_transform(
                   'map',  # Target frame
                   point_msg.header.frame_id,  # Source frame
                   rclpy.time.Time(),  # Time (latest)
                   timeout=rclpy.duration.Duration(seconds=1.0)
               )

               # Perform the transformation
               transformed_point = do_transform_point(point_msg, transform)
               transformed_point.header.frame_id = 'map'
               transformed_point.header.stamp = self.get_clock().now().to_msg()

               # Publish the transformed point
               self.pub.publish(transformed_point)

               self.get_logger().info(
                   f'Transformed point from {point_msg.header.frame_id} to map: '
                   f'({point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f}) -> '
                   f'({transformed_point.point.x:.3f}, {transformed_point.point.y:.3f}, {transformed_point.point.z:.3f})'
               )

           except Exception as e:
               self.get_logger().error(f'Could not transform point: {str(e)}')

       def transform_sample_point(self):
           # Create a sample point in base_link frame
           sample_point = PointStamped()
           sample_point.header.frame_id = 'base_link'
           sample_point.header.stamp = self.get_clock().now().to_msg()
           sample_point.point.x = 1.0  # 1m in front of robot
           sample_point.point.y = 0.5  # 0.5m to the left
           sample_point.point.z = 0.0  # on the ground plane

           # Transform it to map frame
           try:
               transform = self.tf_buffer.lookup_transform(
                   'map',
                   'base_link',
                   rclpy.time.Time(),
                   timeout=rclpy.duration.Duration(seconds=1.0)
               )

               transformed = do_transform_point(sample_point, transform)
               transformed.header.frame_id = 'map'
               transformed.header.stamp = self.get_clock().now().to_msg()

               self.get_logger().info(
                   f'Sample point transformed: base_link({sample_point.point.x:.3f}, {sample_point.point.y:.3f}) -> '
                   f'map({transformed.point.x:.3f}, {transformed.point.y:.3f})'
               )

           except Exception as e:
               self.get_logger().error(f'Could not transform sample point: {str(e)}')

   def main(args=None):
       rclpy.init(args=args)
       transform_converter = TransformConverter()

       try:
           rclpy.spin(transform_converter)
       except KeyboardInterrupt:
           pass
       finally:
           transform_converter.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create a node to generate sample points:
   ```python
   # ~/ros2_ws/src/robot_tf/robot_tf/point_generator.py
   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PointStamped
   import math

   class PointGenerator(Node):
       def __init__(self):
           super().__init__('point_generator')

           # Publisher for points to transform
           self.pub = self.create_publisher(PointStamped, 'input_points', 10)

           # Timer to publish points
           self.timer = self.create_timer(3.0, self.publish_point)
           self.counter = 0

       def publish_point(self):
           # Create a point in different frames to test transformation
           point = PointStamped()

           if self.counter % 3 == 0:
               # Point in base_link
               point.header.frame_id = 'base_link'
               point.point.x = 1.0
               point.point.y = 0.0
               point.point.z = 0.0
           elif self.counter % 3 == 1:
               # Point in laser_frame
               point.header.frame_id = 'laser_frame'
               point.point.x = 0.0
               point.point.y = 0.0
               point.point.z = 1.0  # 1m above laser
           else:
               # Point in odom
               point.header.frame_id = 'odom'
               point.point.x = 2.0
               point.point.y = 1.0
               point.point.z = 0.0

           point.header.stamp = self.get_clock().now().to_msg()
           self.pub.publish(point)

           self.get_logger().info(
               f'Published point in {point.header.frame_id}: '
               f'({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})'
           )

           self.counter += 1

   def main(args=None):
       rclpy.init(args=args)
       point_generator = PointGenerator()

       try:
           rclpy.spin(point_generator)
       except KeyboardInterrupt:
           pass
       finally:
           point_generator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Build and run the transformation nodes:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_tf
   source install/setup.bash

   # Run the transform converter
   ros2 run robot_tf transform_converter

   # In another terminal, run the point generator
   ros2 run robot_tf point_generator
   ```

4. Test with command line tools:
   ```bash
   # Publish a point manually
   ros2 topic pub /input_points geometry_msgs/PointStamped "{
     header: {frame_id: 'base_link'},
     point: {x: 1.0, y: 0.5, z: 0.0}
   }"
   ```

### Expected Outcome
A system that can transform points between different coordinate frames, with proper error handling and logging.

### Troubleshooting Tips
- Ensure TF tree is connected between source and target frames
- Check that timestamps in PointStamped messages are appropriate
- Verify that the transform listener has enough time to populate the buffer

</LabActivity>

## ROS 2 Packages/Tools Used

- `tf2_ros`: ROS 2 TF2 library for transform operations
- `tf2_geometry_msgs`: TF2 extensions for geometry message transformations
- `urdf` and `xacro` packages: Robot description and XML macro language
- `rviz2` for visualization: 3D visualization tool for ROS 2
- `robot_state_publisher`: Publishing robot state transforms from URDF

## Simulation vs Real-Robot Activities

### Simulation:
- TF tree for simulated robot model
- Transform validation in simulation environment

### Real Robot:
- TF calibration and validation with real sensors
- Real-world coordinate frame transformations

## Diagrams and Figures

### TF Tree Visualization Example

```
        map
         |
       odom
         |
     base_link
    /    |    \
laser  imu  wheels
frame  frame   frames
```

### Coordinate Frame Transformation Mathematics

```
P_target = R * P_source + T

Where:
- P_target: Point in target coordinate frame
- P_source: Point in source coordinate frame
- R: Rotation matrix (from quaternion)
- T: Translation vector
```

### URDF to TF Relationship Diagram

```
URDF File
   |
   v
robot_state_publisher
   |
   v
TF Tree in ROS
   |
   v
Transform Lookups
```

## Checklists

### ✓ TF Tree Validation Checklist
- [ ] All frames are connected in a single tree
- [ ] No loops exist in the transform tree
- [ ] Frame names follow ROS naming conventions
- [ ] Transform frequencies are appropriate (10-100 Hz)

### ✓ URDF Model Correctness Checklist
- [ ] All links have proper visual and collision properties
- [ ] Joint definitions are correct with proper parent-child relationships
- [ ] Inertial properties are defined for dynamic simulation
- [ ] Frame IDs match between URDF and code

### ✓ Transform Lookup Accuracy Checklist
- [ ] Lookups use appropriate time stamps
- [ ] Timeout values are reasonable for system performance
- [ ] Error handling implemented for failed lookups
- [ ] Transform caching is used appropriately

## Glossary Terms

- **TF**: Transform library for coordinate frame management
- **Transform**: A relationship between two coordinate frames (position and orientation)
- **Frame**: A coordinate system in the TF tree
- **URDF**: Unified Robot Description Format for robot models
- **Robot State**: Joint positions and transforms for robot links
- **Forward Kinematics**: Calculating end-effector position from joint angles
- **Inverse Kinematics**: Calculating joint angles from end-effector position

## Cross-References

<CrossReference to="/docs/module-1/chapter-4" title="Chapter 4: Parameters and Launch Systems" type="prerequisite">
  Understanding launch systems helps in starting TF-related nodes together.
</CrossReference>

<CrossReference to="/docs/module-1/chapter-6" title="Chapter 6: Real-world Integration and Best Practices" type="continuation">
  Apply TF concepts to real hardware integration and calibration.
</CrossReference>

<CrossReference to="/docs/module-2/chapter-1" title="Module 2, Chapter 1: Introduction to Simulation Environments" type="application">
  Use TF in simulation environments for robot state and sensor integration.
</CrossReference>

## Optional Advanced Section

### TF Interpolation and Buffering Strategies

Advanced techniques for handling transform lookups with proper buffering and interpolation.

### Multi-robot TF Management

Managing coordinate frames for multiple robots operating in the same environment.

<Assessment
  type="assignment"
  title="Chapter 5 Assessment: TF and Navigation Fundamentals"
  objectives={[
    "Create and validate URDF models with proper TF frames",
    "Implement TF broadcasters for robot state",
    "Perform coordinate frame transformations",
    "Visualize and debug TF trees"
  ]}
  rubric={[
    {
      criterion: "URDF Model Creation",
      scores: [
        { label: "Excellent", value: "URDF properly structured with correct frames and properties" },
        { label: "Proficient", value: "URDF functional with minor issues" },
        { label: "Developing", value: "URDF partially correct" },
        { label: "Beginning", value: "URDF not properly structured" }
      ]
    },
    {
      criterion: "TF Broadcasting",
      scores: [
        { label: "Excellent", value: "TF broadcasters properly implemented with dynamic updates" },
        { label: "Proficient", value: "TF broadcasters functional with minor issues" },
        { label: "Developing", value: "TF broadcasters partially implemented" },
        { label: "Beginning", value: "TF broadcasters not properly implemented" }
      ]
    },
    {
      criterion: "Transform Operations",
      scores: [
        { label: "Excellent", value: "Transformations accurate with proper error handling" },
        { label: "Proficient", value: "Transformations work with minor issues" },
        { label: "Developing", value: "Transformations partially functional" },
        { label: "Beginning", value: "Transformations not properly implemented" }
      ]
    }
  ]}
>

### Assignment Tasks

1. **URDF Development**: Create a complete URDF model for a robot with at least 5 links and 4 joints, including proper visual, collision, and inertial properties.

2. **TF Broadcasting**: Implement TF broadcasters that publish transforms based on robot odometry and joint states with proper timing.

3. **Coordinate Transformation**: Create a system that can transform points between multiple coordinate frames with validation and error handling.

4. **TF Visualization**: Use RViz to visualize the TF tree and validate the coordinate frame relationships.

### Submission Requirements

- Complete URDF file for the robot model
- Source code for TF broadcasters
- Documentation of transform validation
- Screenshots of TF visualization in RViz
- Test results showing successful coordinate transformations

</Assessment>

## References

<Citation
  id="ros2-tf2-design-2021"
  authors="Open Robotics"
  year="2021"
  title="ROS 2 TF2 Design"
  source="ROS 2 Documentation"
  url="https://docs.ros.org/en/rolling/Concepts/About-TF2.html"
>
  The official ROS 2 documentation on TF2 explaining coordinate frame management.
</Citation>

<Citation
  id="urdf-specification-2021"
  authors="Open Robotics"
  year="2021"
  title="URDF Specification"
  source="ROS 2 Documentation"
  url="https://wiki.ros.org/urdf/XML"
>
  The URDF specification for robot description format.
</Citation>