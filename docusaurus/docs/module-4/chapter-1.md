---
sidebar_position: 2
---

# Chapter 1: Introduction to Vision-Language-Action Systems

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import CrossReference from '@site/src/components/CrossReference';
import Citation from '@site/src/components/Citation';
import Assessment from '@site/src/components/Assessment';

## Chapter Purpose

This chapter establishes foundational understanding of vision-language-action (VLA) systems and their application to robotics. Students will learn about multimodal integration, the fundamentals of vision-language models for robotics, action space representation and grounding, and the connection between language commands and robotic actions. The chapter emphasizes embodied AI versus conversational AI and sets the groundwork for implementing multimodal perception systems.

<LearningObjectives objectives={[
  "Understand vision-language models and their application to robotics",
  "Learn action space representation and grounding in robotic systems",
  "Integrate multimodal perception systems for robotic decision-making",
  "Connect natural language commands to robotic action planning",
  "Implement basic VLA system architecture with perception-action cycles"
]} />

## Vision-Language-Action Pipelines

- **Input processing**: Combining visual and linguistic modalities for robotic understanding
- **Feature fusion and multimodal representations**: Creating unified representations for decision making
- **Action prediction and execution planning**: Converting multimodal inputs to robotic actions
- **Closed-loop perception-action cycles**: Implementing continuous sensing and acting loops

## Multimodal Perception (Vision + Language Grounding)

- **Visual scene understanding with language context**: Interpreting visual information with linguistic cues
- **Object detection with language grounding**: Identifying objects based on natural language descriptions
- **Spatial reasoning using vision and language**: Understanding spatial relationships through multimodal inputs
- **Cross-modal attention mechanisms**: Focusing on relevant visual elements based on language

## Action Planning and Execution

- **Language-to-action mapping**: Connecting natural language commands to robotic behaviors
- **Task decomposition and sequencing**: Breaking down complex commands into executable steps
- **Robotic skill execution from natural language**: Implementing skills based on language instructions
- **Feedback integration and correction**: Incorporating sensory feedback to refine actions

## LLM-based Robot Reasoning

- **Role of LLMs in VLA systems**: Understanding how large language models contribute to robotic decision making
- **Chain-of-thought reasoning for robotic tasks**: Implementing logical reasoning for complex tasks
- **Knowledge integration and commonsense reasoning**: Incorporating world knowledge for better decision making
- **Planning with language models**: Using LLMs for high-level task planning

## Prompting Strategies for Embodied Agents

- **Effective prompting for robotic tasks**: Crafting prompts that lead to appropriate robotic actions
- **Context-aware prompting techniques**: Adapting prompts based on environmental context
- **Multi-step instruction parsing**: Breaking down complex instructions into manageable steps
- **Error recovery through language**: Using language to handle and recover from errors

## ROS 2 + VLA Integration Points

- **VLA node architecture in ROS 2**: Designing nodes that handle multimodal inputs and outputs
- **Message types for multimodal data**: Defining appropriate message types for vision-language data
- **Integration with existing perception systems**: Connecting VLA systems with traditional perception
- **Communication patterns for VLA systems**: Establishing appropriate communication patterns

## Simulation vs Real-robot Deployment

- **VLA system simulation and testing**: Validating VLA systems in simulation environments
- **Real-world deployment considerations**: Addressing challenges in real-world deployment
- **Transfer from simulated to real environments**: Adapting VLA systems for real-world use
- **Performance validation and comparison**: Evaluating system performance across environments

## Safety, Alignment, and Failure Handling

- **Safety constraints for language-driven actions**: Ensuring safe execution of language commands
- **Alignment with human intent and values**: Aligning robot behavior with human expectations
- **Failure detection and recovery strategies**: Implementing robust failure handling
- **Safe exploration and learning**: Ensuring safe exploration in VLA systems

## Practical Demonstrations

### 1. Basic VLA Pipeline Implementation

Creating a basic vision-language-action pipeline:

```python
# vla_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import torch
import clip
from PIL import Image as PILImage

class VLAPipeline(Node):
    def __init__(self):
        super().__init__('vla_pipeline')

        # Initialize components
        self.cv_bridge = CvBridge()

        # Load CLIP model for vision-language understanding
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.clip_model, self.preprocess = clip.load("ViT-B/32", device=self.device)
        self.clip_model.eval()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )

        # Create publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.latest_image = None
        self.pending_command = None

        self.get_logger().info('VLA Pipeline initialized')

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store for later processing
            self.latest_image = cv_image

            # If we have a pending command, process both
            if self.pending_command:
                self.process_command_and_image(self.pending_command, cv_image)
                self.pending_command = None

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def command_callback(self, msg):
        """Process incoming natural language command"""
        command = msg.data.lower()

        # Store command for processing with next image
        self.pending_command = command

        # If we have a recent image, process both
        if self.latest_image is not None:
            self.process_command_and_image(command, self.latest_image)
            self.pending_command = None

    def process_command_and_image(self, command, image):
        """Process combined vision and language input to generate action"""
        try:
            # Convert image to PIL and preprocess
            pil_image = PILImage.fromarray(image[:, :, ::-1])  # Convert BGR to RGB
            image_input = self.preprocess(pil_image).unsqueeze(0).to(self.device)

            # Tokenize command
            text_input = clip.tokenize([command]).to(self.device)

            # Get similarity between image and text
            with torch.no_grad():
                image_features = self.clip_model.encode_image(image_input)
                text_features = self.clip_model.encode_text(text_input)

                # Compute similarity
                similarity = (image_features @ text_features.T).softmax(dim=-1)
                similarity_score = similarity[0][0].item()

            # Generate action based on command and visual context
            action = self.generate_action_from_command(command, similarity_score)

            # Publish action
            self.publish_action(action)

            self.get_logger().info(f'Processed command "{command}" with similarity {similarity_score:.3f}, action: {action}')

        except Exception as e:
            self.get_logger().error(f'Error in VLA processing: {str(e)}')

    def generate_action_from_command(self, command, similarity_score):
        """Generate appropriate action based on command and visual similarity"""
        # Simple mapping for demonstration
        if 'go to' in command or 'move to' in command:
            if similarity_score > 0.5:
                return 'FORWARD'
            else:
                return 'SEARCH'
        elif 'turn' in command or 'rotate' in command:
            if 'left' in command:
                return 'TURN_LEFT'
            elif 'right' in command:
                return 'TURN_RIGHT'
            else:
                return 'ROTATE'
        elif 'stop' in command:
            return 'STOP'
        elif 'pick up' in command or 'grasp' in command:
            return 'GRASP'
        else:
            return 'UNKNOWN'

    def publish_action(self, action):
        """Convert action to robot command and publish"""
        cmd = Twist()

        if action == 'FORWARD':
            cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        elif action == 'BACKWARD':
            cmd.linear.x = -0.2  # Move backward
        elif action == 'TURN_LEFT':
            cmd.angular.z = 0.5  # Turn left
        elif action == 'TURN_RIGHT':
            cmd.angular.z = -0.5  # Turn right
        elif action == 'STOP':
            cmd.linear.x = 0.0  # Stop
            cmd.angular.z = 0.0
        elif action == 'SEARCH':
            cmd.angular.z = 0.2  # Slow turn to search
        elif action == 'GRASP':
            # In a real robot, this would trigger gripper control
            self.get_logger().info('Grasping action triggered')

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    vla_pipeline = VLAPipeline()

    try:
        rclpy.spin(vla_pipeline)
    except KeyboardInterrupt:
        vla_pipeline.get_logger().info('Shutting down VLA Pipeline...')
    finally:
        vla_pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Vision-Language Model Integration

Integrating vision-language models with robotic systems:

```python
# vision_language_integrator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import torch
import clip
from transformers import AutoTokenizer, AutoModel
import numpy as np

class VisionLanguageIntegrator(Node):
    def __init__(self):
        super().__init__('vision_language_integrator')

        # Initialize components
        self.cv_bridge = CvBridge()

        # Load vision-language model (CLIP for demonstration)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.clip_model, self.preprocess = clip.load("ViT-B/32", device=self.device)
        self.clip_model.eval()

        # Load language model for more complex reasoning
        self.tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")
        self.text_encoder = AutoModel.from_pretrained("bert-base-uncased")
        self.text_encoder.to(self.device)
        self.text_encoder.eval()

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.language_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.language_callback,
            10
        )

        # Publishers
        self.action_pub = self.create_publisher(String, '/robot_action', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_location', 10)

        # State management
        self.current_image = None
        self.current_command = None
        self.command_queue = []

        self.get_logger().info('Vision-Language Integrator initialized')

    def image_callback(self, msg):
        """Process visual input"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image

            # Process any queued commands with this image
            if self.command_queue:
                command = self.command_queue.pop(0)
                self.process_multimodal_input(command, cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def language_callback(self, msg):
        """Process linguistic input"""
        command = msg.data

        if self.current_image is not None:
            # Process immediately if we have an image
            self.process_multimodal_input(command, self.current_image)
        else:
            # Queue command for later processing
            self.command_queue.append(command)
            if len(self.command_queue) > 5:  # Limit queue size
                self.command_queue.pop(0)

    def process_multimodal_input(self, command, image):
        """Process combined visual and linguistic input"""
        try:
            # Encode image
            pil_image = Image.fromarray(image[:, :, ::-1])
            image_input = self.preprocess(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                image_features = self.clip_model.encode_image(image_input)

            # Encode text
            text_tokens = self.tokenizer(command, return_tensors="pt", padding=True, truncation=True)
            text_tokens = {k: v.to(self.device) for k, v in text_tokens.items()}

            with torch.no_grad():
                text_features = self.text_encoder(**text_tokens).last_hidden_state.mean(dim=1)

            # Combine features for decision making
            combined_features = torch.cat([image_features, text_features], dim=1)

            # Generate action based on combined understanding
            action = self.decision_making(combined_features, command)

            # Publish results
            self.publish_results(action, command)

            self.get_logger().info(f'Processed: "{command}" -> {action}')

        except Exception as e:
            self.get_logger().error(f'Error in multimodal processing: {str(e)}')

    def decision_making(self, features, command):
        """Make decisions based on multimodal features"""
        # This is a simplified example
        # In practice, this would involve more sophisticated reasoning

        if 'go to' in command.lower() or 'move to' in command.lower():
            return 'NAVIGATE_TO_TARGET'
        elif 'pick up' in command.lower() or 'grasp' in command.lower():
            return 'PICK_UP_OBJECT'
        elif 'follow' in command.lower():
            return 'FOLLOW_OBJECT'
        elif 'avoid' in command.lower():
            return 'AVOID_OBSTACLE'
        else:
            return 'STANDBY'

    def publish_results(self, action, command):
        """Publish the results of multimodal processing"""
        # Publish action
        action_msg = String()
        action_msg.data = action
        self.action_pub.publish(action_msg)

        # For navigation tasks, also publish target location
        if action == 'NAVIGATE_TO_TARGET':
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = 'map'
            # Set target coordinates based on command interpretation
            target_msg.pose.position.x = 1.0  # Placeholder
            target_msg.pose.position.y = 1.0  # Placeholder
            target_msg.pose.orientation.w = 1.0
            self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    integrator = VisionLanguageIntegrator()

    try:
        rclpy.spin(integrator)
    except KeyboardInterrupt:
        integrator.get_logger().info('Shutting down Vision-Language Integrator...')
    finally:
        integrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Simple Command Interpretation and Execution

Implementing basic command interpretation and action execution:

```python
# command_interpreter.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import re

class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')

        # Create subscribers and publishers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.safety_distance = 0.5  # meters
        self.obstacle_detected = False

        self.get_logger().info('Command Interpreter initialized')

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        if len(msg.ranges) > 0:
            min_distance = min([r for r in msg.ranges if r > 0 and r < float('inf')], default=float('inf'))
            self.obstacle_detected = min_distance < self.safety_distance

    def command_callback(self, msg):
        """Process natural language command"""
        command = msg.data.lower().strip()

        # Parse command and generate action
        action = self.parse_command(command)

        if action:
            # Check safety before executing
            if self.is_safe_to_execute(action):
                self.execute_action(action)
                self.get_logger().info(f'Executed: {action}')
            else:
                self.get_logger().warn(f'Safety check failed for action: {action}')
        else:
            self.get_logger().warn(f'Could not parse command: {command}')

    def parse_command(self, command):
        """Parse natural language command into action"""
        # Simple rule-based parsing for demonstration
        command = command.lower()

        # Navigation commands
        if any(word in command for word in ['go', 'move', 'drive', 'navigate']):
            if any(word in command for word in ['forward', 'ahead', 'straight']):
                return 'MOVE_FORWARD'
            elif any(word in command for word in ['backward', 'back']):
                return 'MOVE_BACKWARD'
            elif any(word in command for word in ['left', 'port']):
                return 'TURN_LEFT'
            elif any(word in command for word in ['right', 'starboard']):
                return 'TURN_RIGHT'

        # Speed commands
        if 'slow' in command:
            return 'SLOW_SPEED'
        elif 'fast' in command or 'quick' in command:
            return 'FAST_SPEED'

        # Stop command
        if any(word in command for word in ['stop', 'halt', 'pause']):
            return 'STOP'

        # Complex commands
        if 'go to' in command or 'move to' in command:
            # Extract target location if possible
            return 'NAVIGATE_TO_LOCATION'

        if 'avoid' in command or 'obstacle' in command:
            return 'AVOID_OBSTACLES'

        return None

    def is_safe_to_execute(self, action):
        """Check if action is safe to execute given current state"""
        if action in ['MOVE_FORWARD', 'FAST_SPEED'] and self.obstacle_detected:
            return False
        return True

    def execute_action(self, action):
        """Execute the parsed action"""
        cmd = Twist()

        if action == 'MOVE_FORWARD':
            cmd.linear.x = 0.3  # m/s
        elif action == 'MOVE_BACKWARD':
            cmd.linear.x = -0.2
        elif action == 'TURN_LEFT':
            cmd.angular.z = 0.4  # rad/s
        elif action == 'TURN_RIGHT':
            cmd.angular.z = -0.4
        elif action == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif action == 'SLOW_SPEED':
            # Modify current command to slow speed
            pass  # This would modify ongoing motion
        elif action == 'FAST_SPEED':
            # Modify current command to fast speed
            pass  # This would modify ongoing motion
        elif action == 'AVOID_OBSTACLES':
            # Implement obstacle avoidance logic
            if self.obstacle_detected:
                cmd.angular.z = 0.5  # Turn away from obstacle
            else:
                cmd.linear.x = 0.2  # Continue forward if no obstacle

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    interpreter = CommandInterpreter()

    try:
        rclpy.spin(interpreter)
    except KeyboardInterrupt:
        interpreter.get_logger().info('Shutting down Command Interpreter...')
    finally:
        interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Coding Labs

<LabActivity title="Lab 4.1: Setting Up Vision-Language Model Integration" time="60 min" difficulty="Hard">

### Objective
Set up vision-language model integration with robotic systems using CLIP or similar models.

### Steps
1. Install required dependencies for vision-language models:
   ```bash
   # Install PyTorch and vision libraries
   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
   pip3 install transformers open_clip_torch pillow numpy

   # Install ROS 2 Python packages
   pip3 install opencv-python cv-bridge
   ```

2. Create a vision-language integration package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python vision_language_integration
   cd vision_language_integration
   mkdir -p vision_language_integration
   ```

3. Create the vision-language model integrator:
   ```python
   # vision_language_integration/vision_language_integration/model_integrator.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   from cv_bridge import CvBridge
   import torch
   import open_clip
   from PIL import Image as PILImage
   import numpy as np

   class VisionLanguageModelIntegrator(Node):
       def __init__(self):
           super().__init__('vision_language_model_integrator')

           # Initialize CV bridge
           self.cv_bridge = CvBridge()

           # Load vision-language model
           self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
           self.get_logger().info(f'Using device: {self.device}')

           # Load CLIP model
           try:
               self.model, _, self.preprocess = open_clip.create_model_and_transforms(
                   'ViT-B-32', pretrained='openai'
               )
               self.model = self.model.to(self.device)
               self.model.eval()
               self.tokenizer = open_clip.get_tokenizer('ViT-B-32')

               self.get_logger().info('Vision-language model loaded successfully')
           except Exception as e:
               self.get_logger().error(f'Failed to load vision-language model: {e}')
               return

           # Create subscribers and publishers
           self.image_sub = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )

           self.command_sub = self.create_subscription(
               String,
               '/natural_language_command',
               self.command_callback,
               10
           )

           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           # Internal state
           self.latest_image_features = None
           self.command_queue = []

           self.get_logger().info('Vision-Language Model Integrator initialized')

       def image_callback(self, msg):
           """Process incoming image and extract features"""
           try:
               # Convert ROS Image to OpenCV
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

               # Convert to PIL and preprocess
               pil_image = PILImage.fromarray(cv_image[:, :, ::-1])  # BGR to RGB
               image_tensor = self.preprocess(pil_image).unsqueeze(0).to(self.device)

               # Extract features
               with torch.no_grad():
                   image_features = self.model.encode_image(image_tensor)
                   image_features /= image_features.norm(dim=-1, keepdim=True)  # Normalize

               self.latest_image_features = image_features

               # Process any queued commands
               while self.command_queue:
                   command = self.command_queue.pop(0)
                   self.process_command_with_image(command, image_features)

           except Exception as e:
               self.get_logger().error(f'Error processing image: {str(e)}')

       def command_callback(self, msg):
           """Process natural language command"""
           command = msg.data

           if self.latest_image_features is not None:
               # Process immediately if we have image features
               self.process_command_with_image(command, self.latest_image_features)
           else:
               # Queue command for later processing
               self.command_queue.append(command)
               if len(self.command_queue) > 5:  # Limit queue size
                   self.command_queue.pop(0)

       def process_command_with_image(self, command, image_features):
           """Process command with current image features"""
           try:
               # Tokenize command
               text_tokens = self.tokenizer([command])
               text_tokens = text_tokens.to(self.device)

               # Get text features
               with torch.no_grad():
                   text_features = self.model.encode_text(text_tokens)
                   text_features /= text_features.norm(dim=-1, keepdim=True)  # Normalize

               # Compute similarity
               similarity = (image_features @ text_features.T).squeeze().item()

               # Generate response based on similarity
               self.generate_response(command, similarity)

           except Exception as e:
               self.get_logger().error(f'Error processing command with image: {str(e)}')

       def generate_response(self, command, similarity):
           """Generate appropriate response based on command and visual similarity"""
           self.get_logger().info(f'Command: "{command}", Similarity: {similarity:.3f}')

           # Generate robot action based on command and similarity
           action = self.interpret_command_and_similarity(command, similarity)

           # Execute action
           self.execute_action(action)

       def interpret_command_and_similarity(self, command, similarity):
           """Interpret command and similarity to determine action"""
           # Simple interpretation for demonstration
           if 'stop' in command.lower():
               return 'STOP'
           elif 'go' in command.lower() or 'move' in command.lower():
               if similarity > 0.3:  # Threshold for confidence
                   return 'FORWARD'
               else:
                   return 'SEARCH'
           elif 'turn' in command.lower():
               if 'left' in command.lower():
                   return 'LEFT'
               elif 'right' in command.lower():
                   return 'RIGHT'
               else:
                   return 'TURN_AROUND'
           else:
               return 'STANDBY'

       def execute_action(self, action):
           """Execute the determined action"""
           cmd = Twist()

           if action == 'FORWARD':
               cmd.linear.x = 0.2
           elif action == 'LEFT':
               cmd.angular.z = 0.5
           elif action == 'RIGHT':
               cmd.angular.z = -0.5
           elif action == 'SEARCH':
               cmd.angular.z = 0.3  # Slow turn to search
           elif action == 'STOP':
               cmd.linear.x = 0.0
               cmd.angular.z = 0.0
           elif action == 'TURN_AROUND':
               cmd.angular.z = 1.0  # Turn around

           self.cmd_vel_pub.publish(cmd)
           self.get_logger().info(f'Executed action: {action}')

   def main(args=None):
       rclpy.init(args=args)
       integrator = VisionLanguageModelIntegrator()

       try:
           rclpy.spin(integrator)
       except KeyboardInterrupt:
           integrator.get_logger().info('Shutting down Vision-Language Model Integrator...')
       finally:
           integrator.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Create a launch file for the integrator:
   ```python
   # launch/vision_language_integrator.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='vision_language_integration',
               executable='model_integrator',
               name='vision_language_model_integrator',
               parameters=[
                   {'use_sim_time': True}
               ],
               output='screen'
           )
       ])
   ```

5. Create setup.py for the package:
   ```python
   # setup.py
   from setuptools import find_packages, setup

   package_name = 'vision_language_integration'

   setup(
       name=package_name,
       version='0.0.0',
       packages=find_packages(exclude=['test']),
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           ('share/' + package_name + '/launch', ['launch/vision_language_integrator.launch.py']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Vision-language model integration for robotics',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'model_integrator = vision_language_integration.model_integrator:main',
           ],
       },
   )
   ```

6. Create package.xml:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>vision_language_integration</name>
     <version>0.0.0</version>
     <description>Vision-language model integration for robotics</description>
     <maintainer email="your.email@example.com">Your Name</maintainer>
     <license>Apache-2.0</license>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>geometry_msgs</depend>
     <depend>cv_bridge</depend>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

7. Build and test the vision-language integration:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select vision_language_integration
   source install/setup.bash

   # Run the vision-language model integrator
   ros2 run vision_language_integration model_integrator
   ```

8. Test with sample commands:
   ```bash
   # In another terminal, send a command
   ros2 topic pub /natural_language_command std_msgs/String "data: 'go forward'"

   # Send an image (if you have a camera running)
   # Or simulate with a test image
   ```

### Expected Outcome
A vision-language model integration system that can process both visual input and natural language commands to generate appropriate robotic actions.

### Troubleshooting Tips
- Ensure PyTorch and vision-language models are properly installed
- Check that the model files are accessible and not corrupted
- Verify that image and command topics are properly connected

</LabActivity>

<LabActivity title="Lab 4.2: Basic Object Recognition with Language Grounding" time="45 min" difficulty="Medium">

### Objective
Implement basic object recognition with language grounding to identify objects based on natural language descriptions.

### Steps
1. Create an object recognition node with language grounding:
   ```python
   # vision_language_integration/vision_language_integration/object_grounding.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from std_msgs.msg import String
   from vision_msgs.msg import Detection2DArray, Detection2D
   from cv_bridge import CvBridge
   import torch
   import open_clip
   from PIL import Image as PILImage
   import numpy as np

   class ObjectGrounding(Node):
       def __init__(self):
           super().__init__('object_grounding')

           # Initialize CV bridge
           self.cv_bridge = CvBridge()

           # Load vision-language model
           self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
           self.get_logger().info(f'Using device: {self.device}')

           try:
               self.model, _, self.preprocess = open_clip.create_model_and_transforms(
                   'ViT-B-32', pretrained='openai'
               )
               self.model = self.model.to(self.device)
               self.model.eval()
               self.tokenizer = open_clip.get_tokenizer('ViT-B-32')

               self.get_logger().info('Object grounding model loaded successfully')
           except Exception as e:
               self.get_logger().error(f'Failed to load object grounding model: {e}')
               return

           # Create subscribers and publishers
           self.image_sub = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )

           self.query_sub = self.create_subscription(
               String,
               '/object_query',
               self.query_callback,
               10
           )

           self.detection_pub = self.create_publisher(Detection2DArray, '/object_detections', 10)

           # Internal state
           self.current_image = None
           self.query_queue = []

           self.get_logger().info('Object Grounding node initialized')

       def image_callback(self, msg):
           """Process incoming image"""
           try:
               # Store image for processing with next query
               cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
               self.current_image = cv_image

               # Process any queued queries
               while self.query_queue:
                   query = self.query_queue.pop(0)
                   self.process_query_with_image(query, cv_image)

           except Exception as e:
               self.get_logger().error(f'Error processing image: {str(e)}')

       def query_callback(self, msg):
           """Process object query"""
           query = msg.data

           if self.current_image is not None:
               # Process immediately if we have an image
               self.process_query_with_image(query, self.current_image)
           else:
               # Queue query for later processing
               self.query_queue.append(query)
               if len(self.query_queue) > 3:  # Limit queue size
                   self.query_queue.pop(0)

       def process_query_with_image(self, query, image):
           """Process object query with current image"""
           try:
               # Convert image to PIL and preprocess
               pil_image = PILImage.fromarray(image[:, :, ::-1])  # BGR to RGB
               image_tensor = self.preprocess(pil_image).unsqueeze(0).to(self.device)

               # Prepare text candidates for the query
               # For demonstration, we'll use the query directly and some variations
               candidates = [
                   query,
                   f"a photo of {query}",
                   f"{query} in a room",
                   f"image of {query}",
                   "background"  # Add background as a candidate
               ]

               # Tokenize candidates
               text_tokens = self.tokenizer(candidates)
               text_tokens = text_tokens.to(self.device)

               # Extract features
               with torch.no_grad():
                   image_features = self.model.encode_image(image_tensor)
                   text_features = self.model.encode_text(text_tokens)

                   # Normalize features
                   image_features /= image_features.norm(dim=-1, keepdim=True)
                   text_features /= text_features.norm(dim=-1, keepdim=True)

                   # Compute similarity
                   similarity = (image_features @ text_features.T).squeeze()
                   probabilities = similarity.softmax(dim=-1)

               # Get the most likely match (excluding background)
               # Exclude the last element which is "background"
               object_probs = probabilities[:-1]  # Exclude background
               best_match_idx = torch.argmax(object_probs).item()
               best_probability = object_probs[best_match_idx].item()

               # Create detection result
               detection_array = Detection2DArray()
               detection_array.header.stamp = self.get_clock().now().to_msg()
               detection_array.header.frame_id = 'camera_frame'

               if best_probability > 0.2:  # Confidence threshold
                   detection = Detection2D()
                   detection.header.stamp = detection_array.header.stamp
                   detection.header.frame_id = detection_array.header.frame_id
                   detection.results = []  # In a real implementation, you'd add classification results

                   # For now, just log the result
                   detected_object = candidates[best_match_idx]
                   self.get_logger().info(f'Detected: "{detected_object}" with confidence {best_probability:.3f}')

                   detection_array.detections.append(detection)
               else:
                   self.get_logger().info(f'No confident match found for query: "{query}"')

               # Publish results
               self.detection_pub.publish(detection_array)

           except Exception as e:
               self.get_logger().error(f'Error in object grounding: {str(e)}')

   def main(args=None):
       rclpy.init(args=args)
       object_grounding = ObjectGrounding()

       try:
           rclpy.spin(object_grounding)
       except KeyboardInterrupt:
           object_grounding.get_logger().info('Shutting down Object Grounding node...')
       finally:
           object_grounding.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Update the package setup to include the new node:
   ```python
   # Add to entry_points in setup.py
   entry_points={
       'console_scripts': [
           'model_integrator = vision_language_integration.model_integrator:main',
           'object_grounding = vision_language_integration.object_grounding:main',
       ],
   },
   ```

3. Create a launch file for object grounding:
   ```python
   # launch/object_grounding.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='vision_language_integration',
               executable='object_grounding',
               name='object_grounding',
               parameters=[
                   {'use_sim_time': True}
               ],
               output='screen'
           )
       ])
   ```

4. Update the package.xml to include new launch files:
   ```xml
   <data_files>
     <path>share/</path>
     <install_to>share/$(name)/launch</install_to>
     <destination>launch</destination>
   </data_files>
   ```

5. Build and run the object grounding node:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select vision_language_integration
   source install/setup.bash

   # Run the object grounding node
   ros2 launch vision_language_integration object_grounding.launch.py
   ```

6. Test with sample queries:
   ```bash
   # In another terminal, send object queries
   ros2 topic pub /object_query std_msgs/String "data: 'red cup'"
   ros2 topic pub /object_query std_msgs/String "data: 'green plant'"
   ros2 topic pub /object_query std_msgs/String "data: 'white chair'"
   ```

### Expected Outcome
A system that can recognize objects in images based on natural language descriptions with confidence scores.

### Troubleshooting Tips
- Ensure the vision-language model is properly loaded
- Check that image and query topics are connected
- Adjust confidence thresholds based on your use case

</LabActivity>

<LabActivity title="Lab 4.3: Simple Command Interpretation and Execution" time="45 min" difficulty="Medium">

### Objective
Implement a simple command interpretation system that converts natural language commands to robotic actions.

### Steps
1. Create a command interpretation node:
   ```python
   # vision_language_integration/vision_language_integration/command_interpreter.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from geometry_msgs.msg import Twist
   from sensor_msgs.msg import LaserScan
   import re
   import json

   class CommandInterpreter(Node):
       def __init__(self):
           super().__init__('command_interpreter')

           # Create subscribers and publishers
           self.command_sub = self.create_subscription(
               String,
               '/natural_language_command',
               self.command_callback,
               10
           )

           self.laser_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.laser_callback,
               10
           )

           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

           # Robot state
           self.safety_distance = 0.5  # meters
           self.obstacle_detected = False
           self.current_speed = 0.2  # default linear speed
           self.current_angular_speed = 0.5  # default angular speed

           # Command mappings
           self.direction_map = {
               'forward': ('linear', 'x', 1),
               'backward': ('linear', 'x', -1),
               'back': ('linear', 'x', -1),
               'ahead': ('linear', 'x', 1),
               'left': ('angular', 'z', 1),
               'right': ('angular', 'z', -1),
               'clockwise': ('angular', 'z', -1),
               'counterclockwise': ('angular', 'z', 1),
               'anti-clockwise': ('angular', 'z', 1)
           }

           # Speed modifiers
           self.speed_modifiers = {
               'slow': 0.5,
               'slowly': 0.5,
               'fast': 2.0,
               'quickly': 2.0,
               'quick': 2.0,
               'rapidly': 2.0
           }

           self.get_logger().info('Command Interpreter initialized')

       def laser_callback(self, msg):
           """Process laser scan for obstacle detection"""
           if len(msg.ranges) > 0:
               valid_ranges = [r for r in msg.ranges if r > 0 and r < float('inf')]
               if valid_ranges:
                   min_distance = min(valid_ranges)
                   self.obstacle_detected = min_distance < self.safety_distance
                   if self.obstacle_detected:
                       self.get_logger().debug(f'Obstacle detected at {min_distance:.2f}m')

       def command_callback(self, msg):
           """Process natural language command"""
           raw_command = msg.data.strip()
           command = raw_command.lower()

           self.get_logger().info(f'Received command: "{raw_command}"')

           # Parse command and generate action
           action = self.parse_command(command)

           if action:
               # Check safety before executing
               if self.is_safe_to_execute(action):
                   self.execute_action(action)
                   self.get_logger().info(f'Executed action: {action}')
               else:
                   self.get_logger().warn(f'Safety check failed for action: {action}')
                   self.handle_unsafe_action(action)
           else:
               self.get_logger().warn(f'Could not parse command: {command}')
               self.publish_feedback(f'Could not understand command: {raw_command}')

       def parse_command(self, command):
           """Parse natural language command into structured action"""
           # Initialize action structure
           action = {
               'type': 'motion',
               'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
               'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
               'duration': 1.0,  # seconds
               'speed_modifier': 1.0
           }

           # Check for stop commands first
           stop_words = ['stop', 'halt', 'pause', 'freeze', 'cease', 'quit']
           if any(word in command for word in stop_words):
               action['type'] = 'stop'
               return action

           # Extract speed modifiers
           for modifier, factor in self.speed_modifiers.items():
               if modifier in command:
                   action['speed_modifier'] = factor
                   break

           # Extract direction and motion type
           found_motion = False
           for direction, (axis, component, sign) in self.direction_map.items():
               if direction in command:
                   if axis == 'linear':
                       action['linear'][component] = sign * self.current_speed
                   elif axis == 'angular':
                       action['angular'][component] = sign * self.current_angular_speed
                   found_motion = True

                   # Look for duration indicators
                   # Simple pattern: move X meters, go for Y seconds
                   meters_match = re.search(r'(\d+(?:\.\d+)?)\s*(?:m(?:eter)?s?)', command)
                   seconds_match = re.search(r'(\d+(?:\.\d+)?)\s*(?:s(?:econd)?s?)', command)

                   if meters_match:
                       distance = float(meters_match.group(1))
                       # Calculate approximate time based on speed (simplified)
                       action['duration'] = distance / abs(action['linear']['x']) if action['linear']['x'] != 0 else 1.0
                   elif seconds_match:
                       action['duration'] = float(seconds_match.group(1))

                   break

           # Apply speed modifier
           for axis in ['linear', 'angular']:
               for component in ['x', 'y', 'z']:
                   action[axis][component] *= action['speed_modifier']

           if found_motion:
               return action
           else:
               # Check for complex commands
               if 'turn around' in command or 'spin around' in command:
                   action['angular']['z'] = self.current_angular_speed
                   action['duration'] = 3.0  # approximately 180 degrees
                   return action
               elif 'look around' in command or 'scan' in command:
                   action['angular']['z'] = self.current_angular_speed * 0.5  # slower turn
                   action['duration'] = 4.0  # scan for 4 seconds
                   return action

           return None  # Could not parse

       def is_safe_to_execute(self, action):
           """Check if action is safe to execute given current state"""
           if action['type'] == 'stop':
               return True  # Stopping is always safe

           # Check if moving forward with obstacle detected
           if (action['linear']['x'] > 0) and self.obstacle_detected:
               return False

           # Check if turning toward obstacle
           if action['angular']['z'] != 0 and self.obstacle_detected:
               # This is a simplification - in reality you'd check direction of turn vs obstacle location
               return True  # For now, turns are allowed even with obstacles

           return True

       def handle_unsafe_action(self, action):
           """Handle unsafe action by publishing warning and stopping"""
           self.get_logger().warn('Unsafe action detected - stopping robot')
           stop_cmd = Twist()
           self.cmd_vel_pub.publish(stop_cmd)
           self.publish_feedback('Action stopped for safety reasons')

       def execute_action(self, action):
           """Execute the parsed action"""
           cmd = Twist()

           if action['type'] == 'stop':
               # Already handled by safety checks
               cmd.linear.x = 0.0
               cmd.angular.z = 0.0
           else:
               cmd.linear.x = action['linear']['x']
               cmd.linear.y = action['linear']['y']
               cmd.linear.z = action['linear']['z']
               cmd.angular.x = action['angular']['x']
               cmd.angular.y = action['angular']['y']
               cmd.angular.z = action['angular']['z']

           # Publish command for the duration
           self.timed_movement(cmd, action['duration'])

       def timed_movement(self, cmd, duration):
           """Execute movement for a specific duration"""
           # For simplicity, we'll just publish the command once
           # In a real implementation, you'd use a timer to stop after duration
           self.cmd_vel_pub.publish(cmd)

           # In a real system, you'd implement timed execution
           # This is a simplified version that just publishes the command

       def publish_feedback(self, message):
           """Publish feedback about command processing"""
           # In a real system, you might publish to a feedback topic
           self.get_logger().info(f'Feedback: {message}')

   def main(args=None):
       rclpy.init(args=args)
       interpreter = CommandInterpreter()

       try:
           rclpy.spin(interpreter)
       except KeyboardInterrupt:
           interpreter.get_logger().info('Shutting down Command Interpreter...')
       finally:
           # Stop robot on shutdown
           cmd = Twist()
           interpreter.cmd_vel_pub.publish(cmd)
           interpreter.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Add the new executable to setup.py:
   ```python
   # Add to entry_points in setup.py
   entry_points={
       'console_scripts': [
           'model_integrator = vision_language_integration.model_integrator:main',
           'object_grounding = vision_language_integration.object_grounding:main',
           'command_interpreter = vision_language_integration.command_interpreter:main',
       ],
   },
   ```

3. Create a launch file for the command interpreter:
   ```python
   # launch/command_interpreter.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='vision_language_integration',
               executable='command_interpreter',
               name='command_interpreter',
               parameters=[
                   {'use_sim_time': True}
               ],
               output='screen'
           )
       ])
   ```

4. Build and run the command interpreter:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select vision_language_integration
   source install/setup.bash

   # Run the command interpreter
   ros2 launch vision_language_integration command_interpreter.launch.py
   ```

5. Test with various commands:
   ```bash
   # In another terminal, send commands
   ros2 topic pub /natural_language_command std_msgs/String "data: 'go forward'"
   ros2 topic pub /natural_language_command std_msgs/String "data: 'turn left'"
   ros2 topic pub /natural_language_command std_msgs/String "data: 'move slowly'"
   ros2 topic pub /natural_language_command std_msgs/String "data: 'stop'"
   ros2 topic pub /natural_language_command std_msgs/String "data: 'go forward quickly'"
   ```

### Expected Outcome
A command interpretation system that converts natural language commands to appropriate robot motion commands with safety checks.

### Troubleshooting Tips
- Verify that the command topic is properly connected
- Check that safety checks are working appropriately
- Adjust speed modifiers and safety distances based on your robot's capabilities

</LabActivity>

## ROS 2 Packages/Tools Used

- `vision_msgs`: Standard messages for vision-based perception
- `sensor_msgs`: Standard messages for sensors including cameras and LiDAR
- `geometry_msgs`: Standard messages for geometric primitives
- `open_clip` or `clip`: Vision-language model libraries
- `transformers`: Hugging Face library for language models
- `torch`: PyTorch for deep learning computations

## Simulation vs Real-Robot Deployment

### Simulation:
- Vision-language model testing with synthetic data
- Language command interpretation in safe virtual environment

### Real Robot:
- Deployment of VLA systems with real sensors and actuators
- Performance validation in real-world scenarios with actual language commands

## Diagrams and Figures

### VLA System Architecture Diagram

```
Natural Language Command → Language Encoder → Feature Fusion → Action Decoder → Robot Actions
                              ↑                                           ↑
                              |                                           |
                           Visual Input ← Vision Encoder ←--------------+
```

### Multimodal Feature Fusion Architecture

```
Visual Features (CNN) ──┐
                        ├──→ [Fusion Layer] → Action Space
Language Features (BERT) ──┘
```

### Vision-Language Grounding Example

```
Input: "Find the red cup"
Visual Scene: [Image with multiple objects]
Processing: Vision + Language → Object Detection
Output: Location of "red cup" in 3D space
```

## Checklists

### ✓ VLA System Setup Verification Checklist
- [ ] Vision-language models properly installed and loaded
- [ ] ROS 2 message types correctly configured
- [ ] Safety constraints properly implemented
- [ ] Basic command interpretation working

### ✓ Multimodal Integration Validation Checklist
- [ ] Visual and linguistic inputs properly synchronized
- [ ] Feature fusion producing meaningful representations
- [ ] Action predictions aligned with input modalities
- [ ] Performance metrics validated

### ✓ Safety and Alignment Validation Checklist
- [ ] Safety constraints enforced for all actions
- [ ] Language interpretation aligned with human intent
- [ ] Error handling and recovery implemented
- [ ] Privacy and ethical considerations addressed

## Glossary Terms

- **Vision-Language-Action (VLA)**: Integrated system connecting vision, language, and robotic actions
- **Multimodal Integration**: Combining multiple sensory modalities for understanding
- **Language Grounding**: Connecting linguistic expressions to perceptual experiences
- **Cross-modal Attention**: Attention mechanism spanning different modalities
- **Embodied AI**: AI systems with physical interaction capabilities
- **Action Space**: Set of possible actions a robot can perform
- **Perception-Action Cycle**: Continuous loop of sensing, interpreting, and acting

## Cross-References

<CrossReference to="/docs/module-1/chapter-1" title="Chapter 1: Introduction to ROS 2 Architecture" type="prerequisite">
  Understanding ROS 2 architecture is essential for VLA system integration.
</CrossReference>

<CrossReference to="/docs/module-3/chapter-1" title="Module 3, Chapter 1: Introduction to NVIDIA Isaac Ecosystem" type="prerequisite">
  AI integration knowledge helps in understanding VLA systems.
</CrossReference>

<CrossReference to="/docs/module-4/chapter-2" title="Chapter 2: Multimodal Perception and Grounding" type="continuation">
  Continue with advanced multimodal perception after basic VLA understanding.
</CrossReference>

## Optional Advanced Section

### Frontier Vision-Language Models (GPT-4V, Gemini, etc.)

Exploring cutting-edge vision-language models for robotics applications.

### Research Directions in Embodied AI and Robotics

Current research trends in vision-language-action systems for robotics.

<Assessment
  type="assignment"
  title="Chapter 1 Assessment: Introduction to Vision-Language-Action Systems"
  objectives={[
    "Set up vision-language model integration with robotic systems",
    "Implement basic object recognition with language grounding",
    "Create command interpretation system for robotic actions",
    "Validate multimodal integration and safety mechanisms"
  ]}
  rubric={[
    {
      criterion: "Vision-Language Integration",
      scores: [
        { label: "Excellent", value: "Models properly integrated with effective feature fusion" },
        { label: "Proficient", value: "Integration functional with minor issues" },
        { label: "Developing", value: "Integration partially implemented" },
        { label: "Beginning", value: "Integration not properly implemented" }
      ]
    },
    {
      criterion: "Object Recognition",
      scores: [
        { label: "Excellent", value: "Accurate object recognition with reliable language grounding" },
        { label: "Proficient", value: "Recognition functional with minor accuracy issues" },
        { label: "Developing", value: "Recognition partially working" },
        { label: "Beginning", value: "Recognition not properly implemented" }
      ]
    },
    {
      criterion: "Command Interpretation",
      scores: [
        { label: "Excellent", value: "Robust command interpretation with safety mechanisms" },
        { label: "Proficient", value: "Command interpretation working with minor issues" },
        { label: "Developing", value: "Command interpretation partially implemented" },
        { label: "Beginning", value: "Command interpretation not properly implemented" }
      ]
    }
  ]}
>

### Assignment Tasks

1. **VLA Setup**: Install and configure vision-language models with robotic systems.

2. **Object Grounding**: Implement object recognition system that can identify objects based on language descriptions.

3. **Command Interpretation**: Create a system that converts natural language commands to robot actions with safety checks.

4. **Integration Testing**: Test the complete VLA system with various commands and scenarios.

### Submission Requirements

- Source code for vision-language integration
- Implementation of object recognition with language grounding
- Command interpretation system with safety checks
- Testing results and performance metrics
- Documentation of the VLA system architecture

</Assessment>

## References

<Citation
  id="vla-systems-2023"
  authors="NVIDIA AI Robotics Team"
  year="2023"
  title="Vision-Language-Action Systems for Robotics"
  source="NVIDIA Technical Reports"
  url="https://arxiv.org/abs/2306.17101"
>
  Research paper on vision-language-action systems for robotics applications.
</Citation>

<Citation
  id="clip-robotics-2022"
  authors="Radford, et al."
  year="2022"
  title="Learning Transferable Visual Models From Natural Language Supervision"
  source="OpenAI Research"
  url="https://arxiv.org/abs/2103.00020"
>
  Original CLIP paper with applications to robotics and vision-language understanding.
</Citation>