# Physical AI Robotics Textbook

Welcome to the "Physical AI Robotics: From Foundations to Advanced Applications" textbook project. This is a comprehensive educational resource designed for advanced undergraduate and graduate students, researchers, and professionals seeking to understand the intersection of artificial intelligence and robotics.

## Overview

This textbook covers four major modules:

1. **Module 1: The Robotic Nervous System (ROS 2)** - Introduction to ROS 2 architecture, nodes, topics, services, and actions
2. **Module 2: The Digital Twin (Gazebo & Unity)** - Simulation environments and digital twin concepts
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac)** - AI integration in robotics using NVIDIA's Isaac ecosystem
4. **Module 4: Vision-Language-Action (VLA)** - Integration of vision, language, and action systems

## Table of Contents

- [Introduction](./docs/intro.md)
- **Module 1: The Robotic Nervous System (ROS 2)**
  - [Chapter 1: Introduction to ROS 2 Architecture](./docs/module-1/chapter-1.md)
  - [Chapter 2: Nodes, Topics, and Messages](./docs/module-1/chapter-2.md)
  - [Chapter 3: Services and Actions](./docs/module-1/chapter-3.md)
  - [Chapter 4: Parameters and Launch Systems](./docs/module-1/chapter-4.md)
  - [Chapter 5: TF and Navigation Fundamentals](./docs/module-1/chapter-5.md)
  - [Chapter 6: Real-world Integration and Best Practices](./docs/module-1/chapter-6.md)
  - [Module Assessment](./docs/module-1/module-assessment.md)
- **Module 2: The Digital Twin (Gazebo & Unity)**
  - [Chapter 1: Introduction to Simulation Environments](./docs/module-2/chapter-1.md)
  - [Chapter 2: Physics Modeling and Dynamics](./docs/module-2/chapter-2.md)
  - [Chapter 3: Sensor Simulation and Calibration](./docs/module-2/chapter-3.md)
  - [Chapter 4: Environment Design and Scenarios](./docs/module-2/chapter-4.md)
  - [Chapter 5: Simulation-to-Reality Transfer](./docs/module-2/chapter-5.md)
  - [Chapter 6: Advanced Simulation Techniques](./docs/module-2/chapter-6.md)
  - [Module Assessment](./docs/module-2/module-assessment.md)
- **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
  - [Chapter 1: Introduction to NVIDIA Isaac Ecosystem](./docs/module-3/chapter-1.md)
  - [Chapter 2: AI-Powered Perception Systems](./docs/module-3/chapter-2.md)
  - [Chapter 3: Reinforcement Learning for Robotics](./docs/module-3/chapter-3.md)
  - [Chapter 4: Learning-Based Control Policies](./docs/module-3/chapter-4.md)
  - [Chapter 5: GPU Optimization and Deployment](./docs/module-3/chapter-5.md)
  - [Chapter 6: Sim-to-Real Transfer and Deployment](./docs/module-3/chapter-6.md)
  - [Module Assessment](./docs/module-3/module-assessment.md)
- **Module 4: Vision-Language-Action (VLA)**
  - [Chapter 1: Introduction to Vision-Language-Action Systems](./docs/module-4/chapter-1.md)
  - [Chapter 2: Multimodal Perception and Grounding](./docs/module-4/chapter-2.md)
  - [Chapter 3: Language-Grounded Action Planning](./docs/module-4/chapter-3.md)
  - [Chapter 4: Large Language Models for Robotics](./docs/module-4/chapter-4.md)
  - [Chapter 5: VLA System Integration and Deployment](./docs/module-4/chapter-5.md)
  - [Chapter 6: Safety, Alignment, and Advanced VLA Applications](./docs/module-4/chapter-6.md)
  - [Module Assessment](./docs/module-4/module-assessment.md)
- **Capstone Project**
  - [Capstone Introduction](./docs/capstone/intro.md)
  - [Project Template](./docs/capstone/project-template.md)
  - [Assessment](./docs/capstone/assessment.md)
- [Conclusion](./docs/conclusion.md)

## Structure

The textbook is organized into modules and chapters with the following components:

- Learning objectives at the beginning of each chapter
- Theoretical foundations and practical applications
- Hands-on labs with step-by-step instructions
- Exercises and assessments with rubrics
- Cross-references between related topics
- Citations following APA format
- Glossary of technical terms

## Development

To run this textbook locally:

1. Clone the repository
2. Navigate to this directory (`docusaurus`)
3. Install dependencies: `npm install`
4. Start the development server: `npm start`

The site will be available at http://localhost:3000

## Building for Production

To build the static site for deployment:

```bash
npm run build
```

## Deployment

This textbook is configured for deployment to GitHub Pages. To deploy:

```bash
GIT_USER=<your-github-username> npm run deploy
```

## Custom Components

This textbook uses several custom Docusaurus components to enhance the learning experience:

- `LearningObjectives` - Displays chapter learning objectives
- `LabActivity` - Structured format for hands-on labs
- `Exercise` - Various types of exercises (labs, assessments, checklists)
- `Assessment` - Quizzes and assignments with rubrics
- `Glossary` - Technical term definitions
- `Citation` - APA-formatted citations
- `CrossReference` - Links to related content
- `NavigationHelper` - Previous/next chapter navigation

## Accessibility

This textbook follows WCAG 2.1 AA accessibility guidelines to ensure it is usable by people with disabilities.

## License

This textbook is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.