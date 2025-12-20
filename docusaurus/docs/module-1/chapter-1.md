---
sidebar_position: 2
---

# Chapter 1: Introduction to ROS 2 Architecture

import LearningObjectives from '@site/src/components/LearningObjectives';
import LabActivity from '@site/src/components/LabActivity';
import Glossary from '@site/src/components/Glossary';
import Citation from '@site/src/components/Citation';
import Assessment from '@site/src/components/Assessment';

## Chapter Purpose

This chapter establishes foundational understanding of ROS 2 architecture, installation procedures, and basic workspace setup. Students will learn about the middleware concepts that underpin ROS 2, the DDS (Data Distribution Service) implementation, and how to properly set up a development environment for ROS 2 applications.

<LearningObjectives objectives={[
  "Explain the core concepts of ROS 2 architecture including nodes, topics, services, and actions",
  "Understand middleware concepts and DDS implementation in ROS 2",
  "Install ROS 2 and set up a proper development environment",
  "Organize ROS 2 packages using the colcon build system",
  "Manage package dependencies and understand the release cycle"
]} />

## Key Concepts

- **Middleware concepts and DDS (Data Distribution Service)**: Understanding how ROS 2 uses DDS as its underlying communication middleware
- **ROS 2 distributions and release cycle**: Learning about different ROS 2 distributions and their support timelines
- **Workspace structure and colcon build system**: Understanding how to organize ROS 2 packages and build them efficiently
- **Package management and dependency resolution**: Learning how to manage ROS 2 packages and their dependencies

## Practical Demonstrations

### 1. Complete ROS 2 Installation on Ubuntu 22.04

```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop-full

# Environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Creating a Basic Workspace with Source, Build, and Install Directories

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 3. Setting up Environment Variables and Sourcing Setup Files

```bash
# Add to ~/.bashrc for persistent sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Hands-on Coding Labs

<LabActivity title="Lab 1.1: Creating and Building a Minimal ROS 2 Package" time="30 min" difficulty="Easy">

### Objective
Create your first ROS 2 package and verify the build process works correctly.

### Steps
1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new Python-based ROS 2 package:
   ```bash
   ros2 pkg create --build-type ament_python my_first_ros2_pkg
   ```

3. Verify the package was created with the proper structure:
   ```bash
   ls -la my_first_ros2_pkg/
   ```

4. Build the workspace to ensure the new package integrates properly:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

5. Source the workspace to make the new package available:
   ```bash
   source install/setup.bash
   ```

### Expected Outcome
A new ROS 2 package is created with proper directory structure and builds successfully without errors.

### Troubleshooting Tips
- If `ros2 pkg create` command is not found, ensure ROS 2 is properly installed and sourced
- If build fails, check that the package name follows ROS naming conventions (no uppercase letters)

</LabActivity>

<LabActivity title="Lab 1.2: Environment Setup Verification with Basic Commands" time="15 min" difficulty="Easy">

### Objective
Verify that your ROS 2 environment is properly configured and all basic commands work.

### Steps
1. Check your ROS 2 installation version:
   ```bash
   ros2 --version
   ```

2. List all available ROS 2 commands:
   ```bash
   ros2 --help
   ```

3. Check the current ROS domain ID:
   ```bash
   echo $ROS_DOMAIN_ID
   ```

4. List any active ROS 2 nodes (should be empty initially):
   ```bash
   ros2 node list
   ```

5. List any active ROS 2 topics (should be empty initially):
   ```bash
   ros2 topic list
   ```

### Expected Outcome
All commands execute successfully without errors, confirming proper ROS 2 installation and environment setup.

### Troubleshooting Tips
- If commands are not found, ensure ROS 2 setup.bash is sourced in your shell profile
- If environment variables are not set, manually source the ROS 2 installation

</LabActivity>

<LabActivity title="Lab 1.3: Workspace Organization Best Practices" time="45 min" difficulty="Medium">

### Objective
Create a well-organized ROS 2 workspace following best practices for package structure and organization.

### Steps
1. Create a multi-package workspace structure:
   ```bash
   mkdir -p ~/ros2_workspace_demo/src
   cd ~/ros2_workspace_demo
   ```

2. Create multiple related packages for a robot system:
   ```bash
   cd src
   ros2 pkg create --build-type ament_python robot_control
   ros2 pkg create --build-type ament_python robot_sensors
   ros2 pkg create --build-type ament_python robot_navigation
   ```

3. Create a proper package.xml file for one of the packages with dependencies:
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>robot_control</name>
     <version>0.0.0</version>
     <description>Robot control package</description>
     <maintainer email="user@example.com">Your Name</maintainer>
     <license>Apache-2.0</license>

     <depend>rclpy</depend>
     <depend>std_msgs</depend>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

4. Build the workspace:
   ```bash
   cd ~/ros2_workspace_demo
   colcon build --packages-select robot_control robot_sensors robot_navigation
   ```

### Expected Outcome
A properly structured workspace with multiple packages that build successfully and follow ROS 2 best practices.

### Troubleshooting Tips
- Ensure package names are unique and follow naming conventions
- Verify all dependencies are properly declared in package.xml

</LabActivity>

## ROS 2 Packages/Tools Used

- `ros-humble-desktop-full`: Complete ROS 2 Humble Hawksbill installation
- `colcon-common-extensions`: Extensions for the colcon build system
- `rosdep` and `vcs` tools: Package dependency management and version control
- `ros2` command line tools: Core ROS 2 command-line interface

## Simulation vs Real-Robot Activities

### Simulation:
- Virtual machine setup with ROS 2
- Using Gazebo simulation environment

### Real Robot:
- Connecting to existing ROS 2-enabled robot for verification
- Testing with actual hardware

## Diagrams and Figures

### ROS 2 Architecture Diagram

```
+---------------------+
|     Application     |
+---------------------+
|    ROS 2 Client     |
|     Libraries       |
+---------------------+
|   DDS Implementation|
| (FastDDS, CycloneDDS)|
+---------------------+
|   Transport Layer   |
| (TCP/IP, UDP, etc.) |
+---------------------+
```

### Workspace Directory Structure Visualization

```
~/ros2_ws/
├── src/          # Source packages
│   ├── package1/
│   ├── package2/
│   └── ...
├── build/        # Build artifacts
├── install/      # Installation space
└── log/          # Build logs
```

### Installation Flowchart

```
Start
  ↓
Check System Requirements
  ↓
Install Dependencies
  ↓
Add ROS 2 Repository
  ↓
Install ROS 2 Packages
  ↓
Setup Environment
  ↓
Verify Installation
```

## Checklists

### ✓ ROS 2 Installation Verification Checklist
- [ ] ROS 2 command line tools accessible (`ros2 --version`)
- [ ] Environment variables properly set
- [ ] Package installation successful
- [ ] Basic commands working (`ros2 topic list`, etc.)

### ✓ Workspace Setup Validation Checklist
- [ ] Workspace directory structure created
- [ ] Source directory contains packages
- [ ] Build process completes without errors
- [ ] Installation space properly populated

### ✓ Environment Configuration Checklist
- [ ] Setup files sourced in shell profile
- [ ] ROS_DOMAIN_ID configured appropriately
- [ ] Package path environment variables set
- [ ] Commands accessible from any directory

## Glossary Terms

- **DDS (Data Distribution Service)**: A standardized middleware protocol that provides data-centric connectivity between applications
- **Node**: A process that performs computation in ROS
- **Topic**: A named bus over which nodes exchange messages
- **Service**: A synchronous request/response communication pattern
- **Action**: An asynchronous request/response communication pattern with feedback
- **Workspace**: A directory containing ROS 2 packages organized for building
- **Package**: The smallest unit of code organization in ROS
- **Message**: A data structure exchanged between nodes
- **Interface**: The definition of messages, services, or actions

## Optional Advanced Section

### Custom DDS Vendor Configuration

ROS 2 supports multiple DDS implementations including:
- **FastDDS**: Default in ROS 2 Humble
- **CycloneDDS**: Lightweight alternative
- **RTI Connext DDS**: Commercial implementation

### Cross-compilation for Embedded Systems

Setting up ROS 2 for embedded systems requires cross-compilation toolchains and appropriate target architecture configuration.

<Glossary terms={[
  {
    name: "DDS (Data Distribution Service)",
    definition: "A standardized middleware protocol that provides data-centric connectivity between applications"
  },
  {
    name: "Node",
    definition: "A process that performs computation in ROS"
  },
  {
    name: "Topic",
    definition: "A named bus over which nodes exchange messages"
  },
  {
    name: "Service",
    definition: "A synchronous request/response communication pattern"
  },
  {
    name: "Action",
    definition: "An asynchronous request/response communication pattern with feedback"
  },
  {
    name: "Workspace",
    definition: "A directory containing ROS 2 packages organized for building"
  },
  {
    name: "Package",
    definition: "The smallest unit of code organization in ROS"
  },
  {
    name: "Message",
    definition: "A data structure exchanged between nodes"
  },
  {
    name: "Interface",
    definition: "The definition of messages, services, or actions"
  }
]} />

## References

<Citation
  id="ros2-design-2015"
  authors="Faust, A., Tso, K., & Srinivasa, S."
  year="2015"
  title="ROS 2 Design Overview"
  source="Open Robotics"
  url="https://design.ros2.org/"
>
  The official ROS 2 design documentation that explains the architecture decisions and motivation for ROS 2.
</ Citation>

<Citation
  id="dds-spec-2015"
  authors="Object Management Group"
  year="2015"
  title="Data Distribution Service (DDS) Specification"
  source="OMG"
  url="https://www.omg.org/spec/DDS/"
>
  The official specification for the Data Distribution Service middleware used by ROS 2.
</ Citation>

<Assessment
  type="assignment"
  title="Chapter 1 Assessment: ROS 2 Fundamentals"
  objectives={[
    "Install ROS 2 Humble Hawksbill on your development system",
    "Create and build a basic ROS 2 workspace",
    "Verify the installation by running basic ROS 2 commands",
    "Create a simple ROS 2 package using proper naming conventions"
  ]}
  rubric={[
    {
      criterion: "Installation and Setup",
      scores: [
        { label: "Excellent", value: "ROS 2 installed correctly with all dependencies" },
        { label: "Proficient", value: "ROS 2 installed with minor configuration issues" },
        { label: "Developing", value: "ROS 2 installed but with some errors" },
        { label: "Beginning", value: "Installation incomplete or unsuccessful" }
      ]
    },
    {
      criterion: "Workspace Creation",
      scores: [
        { label: "Excellent", value: "Workspace created with proper structure and organization" },
        { label: "Proficient", value: "Workspace created with minor structural issues" },
        { label: "Developing", value: "Workspace created but with organization issues" },
        { label: "Beginning", value: "Workspace not created or structure incorrect" }
      ]
    },
    {
      criterion: "Package Development",
      scores: [
        { label: "Excellent", value: "Package created with proper package.xml and setup.py" },
        { label: "Proficient", value: "Package created with minor configuration issues" },
        { label: "Developing", value: "Package created but with missing elements" },
        { label: "Beginning", value: "Package not created or major elements missing" }
      ]
    }
  ]}
>

### Assignment Tasks

1. **ROS 2 Installation**: Complete the ROS 2 installation following the procedures outlined in the practical demonstrations section. Verify your installation by running `ros2 --version` and ensuring all basic commands are available.

2. **Workspace Setup**: Create a new ROS 2 workspace with at least two packages following the best practices outlined in Lab 1.3. Build the workspace using `colcon build` and verify it completes without errors.

3. **Environment Configuration**: Configure your shell profile to automatically source the ROS 2 setup script and your workspace setup script. Verify that ROS 2 commands are available in new terminal sessions.

4. **Package Creation**: Create a new ROS 2 package with proper structure, including a `package.xml` file with appropriate dependencies and a `setup.py` file for Python packages or `CMakeLists.txt` for C++ packages.

### Submission Requirements

- Provide screenshots of successful installation verification
- Include the output of `colcon build` command showing successful build
- Submit your package.xml and setup.py files for review
- Document any challenges encountered and how they were resolved

</Assessment>