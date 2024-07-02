# <div align="center">GU-Systems</div>

<div align="center">
  <img src="https://github.com/SamthinkGit/GU-systems/assets/92941012/62b27a54-c44e-4fca-92ae-12294dd6d1ee" alt="Software Architecture Diagram" width=700>

|       |                                                                                                                                                                        |
| ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Code  | ![GitHub](https://img.shields.io/badge/License-Propietary-blue) ![GitHub](https://img.shields.io/badge/Version-0.0.1-blue) ![GitHub](https://img.shields.io/badge/release-in_process-orange) ![GitHub](https://img.shields.io/badge/docs-Doxygen-white) ![GitHub](https://img.shields.io/badge/Python-3.10-green) ![GitHub](https://img.shields.io/badge/LLM-OpenAI-white) |
| CI/CD | ![GitHub](https://img.shields.io/badge/style-flake8-green) ![GitHub](https://img.shields.io/badge/build-passing-green)                                                 |
| Wiki  | <div align="center"><a href="https://github.com/SamthinkGit/GU-systems/wiki"><strong>📖 Access our Wiki for detailed documentation</strong></a></div>                  |
</div>




## 🚀 Introduction

Welcome to **(codename) GU Systems**, where the future of human-computer interaction unfolds. Imagine your computer uses clicks and keystrokes but now controlled through an AI-powered interface that understands your intent and seamlessly translates it into actions. This is GU Systems - an avant-garde software/hardware suite driven by intelligent AI to elevate your computing to the next level.

## 🎯 Purpose

GU Systems is an approximation to a ECM (Execution Cognition Machine), it deploys multiple layers to interconect Cognition with Execution, thus to fully understand the user, proccess, think, and control a computer as a human-AI. Automation of complex tasks, plannification and AI assistance are the main goals of this repository.

</div>
<div align="center">
  <img src="https://github.com/SamthinkGit/GU-systems/assets/92941012/f329e545-a2c2-4acc-aedf-d7198e82e16d" alt="GU Systems - Terminal example" width=700>
</div>

## 🛠️ Installation
This project requires both **ROS2** and **colcon build** for compilation. We are working on removing this dependency on the future. For now ensure these dependencies are installed before running [build.sh](/scripts/build.sh). For installation guides, refer to:

- ROS2: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- Colcon: [Colcon Tutorial](https://colcon.readthedocs.io/en/released/user/installation.html)

*Note:* Colcon command generally you can use `colcon -h` for checking if it is already installed

**Build Repository**
```bash
git clone https://github.com/SamthinkGit/GU-systems.git
cd GU-Systems
pip install -r requirements.txt
./scripts/build.sh
```

**Set up the development environment**
```bash
# You can add this line to your ~/.bashrc so you avoid using it each time
source scripts/dev_setup.sh
```
**Set up OPENAI**
```bash
# You can get your OPENAI_API_KEY here: https://www.howtogeek.com/885918/how-to-get-an-openai-api-key/
# TIP: You can also add this line to your ~/.bashrc
export OPENAI_API_KEY='sk-...'
```

This command will start executing RePlan in your computer
## ✨ Features

- **Task Planning**: Generate action plans from user requests.
- **I/O Interface**: Interact via fundamental I/O methods.
- **Screen Recognition**: Analyze screen content to assess system status.
- **Command Execution**: Perform necessary background commands.
- **Graphical Elements**: Craft GUI components for user interaction.
- **Hybrid Model**: Harness open-source interfaces for communal growth.

## 📐 Architecture Overview

The ECM architecture is designed to synchronize multiple layers, ensuring efficient, coherent, and functional AI systems. The architecture separates cognitive interactions and decision-making from the practical execution and management of specific functions, improving organization and scalability. The primary layers include:

- **[`/cognition_layer`](/cognition_layer)**: Contains AI agents for thinking, planning, and reasoning, adhering to the Agent Protocol Standard. This layer builds and deploys agents responsible for cognitive tasks.
- **[`/ecm`](/ecm)**: Hosts mediators, communicators, and middleware facilitating interaction between the cognition and execution layers. It includes templates and virtual functions like the Interpreter and ExecutionLayerWrapper, as well as tools usable by both layers.
- **[`/execution_layer`](/execution_layer)**: Includes modules assisting agents in executing commands, typically using the Exelent language. This layer manages thread operations, registries, and callbacks.
- **[`/action_space`](/action_space)**: Defines all actions that agents can perform. By importing these modules, agents gain new interaction capabilities, with actions handled by the execution layer. This includes keyboard interaction, window management, and more.

The ECM architecture is visually represented in the following diagram, highlighting the flow of interactions between the cognition, mediation, and execution layers. This diagram illustrates the separation of responsibilities, with upper layers focusing on cognitive processes and lower layers managing practical execution and functions.

</div>
<div align="center">
  <img src="https://github.com/SamthinkGit/GU-systems/assets/92941012/e3e96550-110e-4bc5-8478-81d85638de31" alt="ECM Layers Diagram" width=450>
</div>

## 📘 Documentation
API Docs: Powered by Doxygen for a deep dive into the codebase.

## 📋 Style Guidelines
Please make sure to pass the **flake8** checks for styling conformity, we use [.precommit-config.yaml](/.precommit-config.yaml) file to auto-check files with flake8 linting and good code practices. Commits that do not pass this test can't be pushed.

## 📜 License
Distributed under a private License. See LICENSE for more information.

<div align="center">
  Innovate, Integrate, and Elevate your experience with GU Systems.
</div>
