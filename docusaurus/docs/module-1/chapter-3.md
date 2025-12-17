---
sidebar_position: 4
---

# Chapter 3: Services and Actions

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import CrossReference from '@site/src/components/CrossReference';
import Citation from '@site/src/components/Citation';
import Assessment from '@site/src/components/Assessment';

## Chapter Purpose

This chapter covers advanced communication patterns in ROS 2, specifically services for synchronous request/response communication and actions for goal-oriented asynchronous communication with feedback. Students will learn to implement service servers and clients, action servers and clients, and understand when to use each communication pattern based on system requirements.

<LearningObjectives objectives={[
  "Implement service-based communication for synchronous requests in ROS 2",
  "Design and implement action servers for long-running goals with feedback",
  "Create service and action clients to interact with servers",
  "Understand feedback and result handling in actions",
  "Choose appropriate communication patterns based on system requirements"
]} />

## Key Concepts

- **Service-based communication for synchronous requests**: Understanding the request/response pattern for immediate results
- **Action-based communication for long-running goals**: Implementing goal-oriented communication with feedback and status updates
- **Client-server interaction patterns**: Designing proper client-server architectures in ROS 2
- **Feedback and result handling in actions**: Managing intermediate feedback and final results in action-based communication

## Practical Demonstrations

### 1. Implementing a Simple Service for Robot Control

Creating a service that allows other nodes to set robot parameters:

```python
# robot_control_service.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        self.srv = self.create_service(
            SetBool,
            'robot_control',
            self.control_callback
        )
        self.get_logger().info('Robot control service is ready')

    def control_callback(self, request, response):
        if request.data:
            self.get_logger().info('Robot activated')
            response.success = True
            response.message = 'Robot activated successfully'
        else:
            self.get_logger().info('Robot deactivated')
            response.success = True
            response.message = 'Robot deactivated successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    robot_control_service = RobotControlService()
    rclpy.spin(robot_control_service)
    robot_control_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Creating Action Server for Trajectory Execution

Implementing an action server for executing robot trajectories:

```python
# trajectory_action_server.py
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('trajectory_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'execute_trajectory',
            self.execute_trajectory
        )

    def execute_trajectory(self, goal_handle):
        self.get_logger().info('Executing trajectory...')

        # Feedback and result messages
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate trajectory execution
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Trajectory execution canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # Simulate work

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Trajectory execution succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    trajectory_action_server = TrajectoryActionServer()
    rclpy.spin(trajectory_action_server)
    trajectory_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Client Implementation for Service and Action Calls

Creating clients to interact with services and actions:

```python
# service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class RobotControlClient(Node):
    def __init__(self):
        super().__init__('robot_control_client')
        self.cli = self.create_client(SetBool, 'robot_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_request(self, state):
        self.req.data = state
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    robot_control_client = RobotControlClient()

    # Activate robot
    response = robot_control_client.send_request(True)
    print(f'Response: {response.success}, {response.message}')

    # Deactivate robot
    response = robot_control_client.send_request(False)
    print(f'Response: {response.success}, {response.message}')

    robot_control_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Coding Labs

<LabActivity title="Lab 3.1: Building a Service to Set Robot Parameters" time="45 min" difficulty="Medium">

### Objective
Create a ROS 2 service that allows other nodes to set robot parameters such as speed, direction, or operational mode.

### Steps
1. Create a new service definition file (or use existing ones like SetBool, SetInt, etc.)

2. Create the service server node:
   ```python
   # parameter_service.py
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import SetBool, SetInt64

   class RobotParameterService(Node):
       def __init__(self):
           super().__init__('robot_parameter_service')
           self.srv = self.create_service(
               SetBool,
               'set_robot_mode',
               self.set_mode_callback
           )
           self.current_mode = False
           self.get_logger().info('Robot parameter service ready')

       def set_mode_callback(self, request, response):
           self.current_mode = request.data
           response.success = True
           response.message = f'Robot mode set to {"active" if request.data else "inactive"}'
           self.get_logger().info(f'Set robot mode: {request.data}')
           return response

   def main(args=None):
       rclpy.init(args=args)
       robot_parameter_service = RobotParameterService()
       rclpy.spin(robot_parameter_service)
       robot_parameter_service.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Build and run the service:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select your_package_name
   source install/setup.bash
   ros2 run your_package_name parameter_service
   ```

4. Test the service from command line:
   ```bash
   ros2 service call /set_robot_mode example_interfaces/srv/SetBool "{data: true}"
   ```

### Expected Outcome
A service that accepts requests to set robot parameters and responds with success status and a descriptive message.

### Troubleshooting Tips
- Ensure the service name is unique and follows ROS naming conventions
- Verify that the service definition is properly installed

</LabActivity>

<LabActivity title="Lab 3.2: Implementing an Action Server for Trajectory Execution" time="60 min" difficulty="Hard">

### Objective
Create a ROS 2 action server that executes robot trajectories with feedback on progress.

### Steps
1. Create the action server with feedback mechanism:
   ```python
   # trajectory_executor.py
   import time
   import rclpy
   from rclpy.action import ActionServer, CancelResponse
   from rclpy.node import Node
   from example_interfaces.action import Fibonacci

   class TrajectoryExecutor(Node):
       def __init__(self):
           super().__init__('trajectory_executor')
           self._action_server = ActionServer(
               self,
               Fibonacci,
               'execute_trajectory',
               self.execute_trajectory,
               cancel_callback=self.cancel_trajectory
           )

       def cancel_trajectory(self, goal_handle):
           self.get_logger().info('Received cancel request')
           return CancelResponse.ACCEPT

       def execute_trajectory(self, goal_handle):
           self.get_logger().info(f'Executing trajectory with order: {goal_handle.request.order}')

           feedback_msg = Fibonacci.Feedback()
           feedback_msg.sequence = [0, 1]

           for i in range(1, goal_handle.request.order):
               if goal_handle.is_cancel_requested:
                   goal_handle.canceled()
                   self.get_logger().info('Trajectory execution canceled')
                   return Fibonacci.Result()

               feedback_msg.sequence.append(
                   feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
               )

               goal_handle.publish_feedback(feedback_msg)
               self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

               time.sleep(0.5)  # Simulate trajectory execution time

           goal_handle.succeed()
           result = Fibonacci.Result()
           result.sequence = feedback_msg.sequence
           self.get_logger().info('Trajectory execution completed successfully')
           return result

   def main(args=None):
       rclpy.init(args=args)
       trajectory_executor = TrajectoryExecutor()
       rclpy.spin(trajectory_executor)
       trajectory_executor.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Build and run the action server:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select your_package_name
   source install/setup.bash
   ros2 run your_package_name trajectory_executor
   ```

3. Test the action from command line:
   ```bash
   ros2 action send_goal /execute_trajectory example_interfaces/action/Fibonacci "{order: 5}"
   ```

### Expected Outcome
An action server that accepts trajectory goals, provides feedback during execution, and returns results when completed.

### Troubleshooting Tips
- Ensure proper handling of cancel requests
- Check that feedback is published at appropriate intervals

</LabActivity>

<LabActivity title="Lab 3.3: Creating Clients to Interact with Services and Actions" time="45 min" difficulty="Medium">

### Objective
Create client nodes that can interact with the services and actions implemented in previous labs.

### Steps
1. Create a service client:
   ```python
   # parameter_client.py
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import SetBool

   class RobotParameterClient(Node):
       def __init__(self):
           super().__init__('robot_parameter_client')
           self.cli = self.create_client(SetBool, 'set_robot_mode')
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')

           self.req = SetBool.Request()

       def send_request(self, state):
           self.req.data = state
           self.future = self.cli.call_async(self.req)
           return self.future

   def main(args=None):
       rclpy.init(args=args)
       robot_parameter_client = RobotParameterClient()

       # Send request to activate robot
       future = robot_parameter_client.send_request(True)
       rclpy.spin_until_future_complete(robot_parameter_client, future)
       response = future.result()
       print(f'Activation response: {response.success}, {response.message}')

       # Send request to deactivate robot
       future = robot_parameter_client.send_request(False)
       rclpy.spin_until_future_complete(robot_parameter_client, future)
       response = future.result()
       print(f'Deactivation response: {response.success}, {response.message}')

       robot_parameter_client.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Create an action client:
   ```python
   # trajectory_client.py
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from example_interfaces.action import Fibonacci

   class TrajectoryClient(Node):
       def __init__(self):
           super().__init__('trajectory_client')
           self._action_client = ActionClient(
               self,
               Fibonacci,
               'execute_trajectory'
           )

       def send_goal(self, order):
           goal_msg = Fibonacci.Goal()
           goal_msg.order = order

           self._action_client.wait_for_server()
           self._send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback
           )

           self._send_goal_future.add_done_callback(self.goal_response_callback)

       def goal_response_callback(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal rejected')
               return

           self.get_logger().info('Goal accepted')
           self._get_result_future = goal_handle.get_result_async()
           self._get_result_future.add_done_callback(self.get_result_callback)

       def feedback_callback(self, feedback_msg):
           self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')

       def get_result_callback(self, future):
           result = future.result().result
           self.get_logger().info(f'Result: {result.sequence}')

   def main(args=None):
       rclpy.init(args=args)
       trajectory_client = TrajectoryClient()

       trajectory_client.send_goal(5)
       rclpy.spin(trajectory_client)

       trajectory_client.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Expected Outcome
Client nodes that can successfully interact with services and actions, sending requests/goals and receiving responses/results.

### Troubleshooting Tips
- Ensure the client waits for the service/action server to be available
- Handle asynchronous calls properly with callbacks

</LabActivity>

## ROS 2 Packages/Tools Used

- `example_interfaces`: Standard ROS 2 interfaces for services and actions
- `rclpy.action` and `rclcpp::action`: ROS 2 client libraries for action communication
- `ros2 service` and `ros2 action` tools: Command line tools for service and action management

## Simulation vs Real-Robot Activities

### Simulation:
- Service calls for simulation control
- Action-based navigation commands to simulated robots

### Real Robot:
- Action-based navigation commands to real robots
- Service calls for hardware configuration and control

## Diagrams and Figures

### Service Communication Sequence Diagram

```
Client          Server
  |                |
  |---request----->|
  |                | Process request
  |                | (blocking)
  |<---response----|
  |                |
```

### Action Lifecycle Diagram with Feedback

```
Client           Server
  |                 |
  |---goal--------->|
  |                 | Accept goal
  |<--accepted------|
  |                 | Execute goal
  |<--feedback------| (repeated)
  |                 |
  |<--result--------| (final)
  |                 |
```

### Client-Server Interaction Patterns

- **Services**: Synchronous request-response, immediate result
- **Actions**: Asynchronous goal-feedback-result, long-running operations
- **Topics**: Asynchronous publisher-subscriber, continuous data stream

## Checklists

### ✓ Service Interface Definition Checklist
- [ ] Service definition properly structured with request and response
- [ ] Dependencies properly declared in package.xml
- [ ] Service interface tested and validated

### ✓ Action Interface Definition Checklist
- [ ] Action definition properly structured with goal, feedback, and result
- [ ] Appropriate data types selected for each component
- [ ] Action interface tested and validated

### ✓ Client Implementation Validation Checklist
- [ ] Client properly waits for service/action server
- [ ] Asynchronous calls handled with appropriate callbacks
- [ ] Error handling implemented for failed requests

## Glossary Terms

- **Service**: A synchronous request/response communication pattern in ROS 2
- **Action**: An asynchronous request/response communication pattern with feedback and status updates
- **Request**: The data structure sent from client to server in service communication
- **Response**: The data structure sent from server to client in service communication
- **Goal**: The initial request sent to an action server
- **Feedback**: Intermediate updates sent from action server during execution
- **Result**: The final outcome sent from action server when execution completes
- **Client**: A ROS 2 node that sends requests to services or goals to actions
- **Server**: A ROS 2 node that processes service requests or action goals

## Cross-References

<CrossReference to="/docs/module-1/chapter-2" title="Chapter 2: Nodes, Topics, and Messages" type="prerequisite">
  Understanding topics is essential before learning about services and actions.
</CrossReference>

<CrossReference to="/docs/module-1/chapter-4" title="Chapter 4: Parameters and Launch Systems" type="continuation">
  After mastering services and actions, learn about parameters and system orchestration.
</CrossReference>

<CrossReference to="/docs/module-3/chapter-1" title="Module 3, Chapter 1: Introduction to NVIDIA Isaac Ecosystem" type="application">
  Apply service and action patterns in AI-robot systems for control and coordination.
</CrossReference>

## Optional Advanced Section

### Multi-threaded Service Servers

Implementing service servers that can handle multiple requests concurrently.

### Custom Action Goal Preemption Logic

Advanced action server implementation with goal preemption and queue management.

<Assessment
  type="assignment"
  title="Chapter 3 Assessment: Services and Actions"
  objectives={[
    "Implement a service server for robot control",
    "Create an action server for long-running goals",
    "Develop clients to interact with services and actions",
    "Handle feedback and results appropriately"
  ]}
  rubric={[
    {
      criterion: "Service Implementation",
      scores: [
        { label: "Excellent", value: "Service fully functional with proper error handling" },
        { label: "Proficient", value: "Service functional with minor issues" },
        { label: "Developing", value: "Service partially implemented" },
        { label: "Beginning", value: "Service not properly implemented" }
      ]
    },
    {
      criterion: "Action Implementation",
      scores: [
        { label: "Excellent", value: "Action server with feedback fully functional" },
        { label: "Proficient", value: "Action server functional with minor issues" },
        { label: "Developing", value: "Action server partially implemented" },
        { label: "Beginning", value: "Action server not properly implemented" }
      ]
    },
    {
      criterion: "Client Development",
      scores: [
        { label: "Excellent", value: "Clients properly handle async communication" },
        { label: "Proficient", value: "Clients functional with minor issues" },
        { label: "Developing", value: "Clients partially implemented" },
        { label: "Beginning", value: "Clients not properly implemented" }
      ]
    }
  ]}
>

### Assignment Tasks

1. **Service Development**: Create a service that allows setting robot speed parameters with validation and error handling.

2. **Action Implementation**: Implement an action server for path planning that provides feedback on progress and returns a complete path when finished.

3. **Client Creation**: Develop client nodes that interact with both the service and action, handling responses and feedback appropriately.

4. **Integration Testing**: Test the service and action in a simulated environment to verify proper communication patterns.

### Submission Requirements

- Source code for all service, action, and client implementations
- Screenshots of successful service and action calls
- Documentation of the communication patterns used
- Reflection on when to use services vs. actions vs. topics

</Assessment>

## References

<Citation
  id="ros2-services-actions-2021"
  authors="Open Robotics"
  year="2021"
  title="ROS 2 Services and Actions Design"
  source="ROS 2 Documentation"
  url="https://docs.ros.org/en/rolling/Concepts/About-Actions.html"
>
  The official ROS 2 documentation on services and actions explaining their design and usage.
</Citation>