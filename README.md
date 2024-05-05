# <div align="center">GU-Systems</div>

<div align="center">

![GitHub](https://img.shields.io/badge/License-Propietary-blue)
![GitHub](https://img.shields.io/badge/Version-0.0.1-blue)
![GitHub](https://img.shields.io/badge/build-passing-green)
![GitHub](https://img.shields.io/badge/style-flake8-green)
![GitHub](https://img.shields.io/badge/docs-Doxygen-white)
![GitHub](https://img.shields.io/badge/release-in_process-orange)


</div>
<div align="center">
  <img src="https://github.com/SamthinkGit/GU-systems/assets/92941012/62b27a54-c44e-4fca-92ae-12294dd6d1ee" alt="Software Architecture Diagram" width=900>
</div>

## 🚀 Introduction

Welcome to **(codename) GU Systems**, where the future of human-computer interaction unfolds. Imagine orchestrating your computer not by mere clicks and keystrokes but through an AI-powered interface that understands your intent and seamlessly translates it into actions. This is GU Systems - an avant-garde software/hardware suite driven by intelligent AI to elevate your computing to the next level.

## 🎯 Purpose

GU Systems propels AI out of its traditional confines, deploying it as a proactive intermediary that crafts plans, understands user requests, and executes them with precision and intuition. Our system is the nexus of innovation, where the AI isn't just on your computer—it's an integral part of your interaction with it.

## 🛠️ Installation
This project requires both **ROS2** and **colcon build** for compilation. Ensure these dependencies are installed before running [build.sh](/scripts/build.sh). For installation guides, refer to:

- ROS2: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- Colcon: [Colcon Tutorial](https://colcon.readthedocs.io/en/released/user/installation.html)

*Note:* Colcon command generally you can use `colcon -h` for checking if it is already installed

**Build Repository**
```bash
git clone https://github.com/SamthinkGit/GU-systems.git
cd GU-Systems
pip install -r requirements.txt
./scripts/interfaces.sh
```

**Set up the development environment**
```bash
# You can also add this line to your ~/.bashrc so you avoid using it each time
source scripts/dev_setup.sh
```

## ✨ Features

- **Task Planning**: Generate action plans from user requests.
- **I/O Interface**: Interact via fundamental I/O methods.
- **Screen Recognition**: Analyze screen content to assess system status.
- **Command Execution**: Perform necessary background commands.
- **Graphical Elements**: Craft GUI components for user interaction.
- **Hybrid Model**: Harness open-source interfaces for communal growth.

## 🧱 Repository Structure
Directory	Content Description
/gusyscore Core python modules for the system
/gusysros	ROS2 workspaces (Mainly Action Nodes) and testing tools
/scripts Bash based scripts for developers

## 📐 Architecture Overview

GU Systems is architecturally a symphony of software and hardware layers, where each note is a purpose-driven component that harmoniously interacts to create a seamless user experience. A general component architecture used for the design of the repository is the following:
<div align="center">
    <img src="https://github.com/SamthinkGit/GU-systems/assets/92941012/4a8b163d-6654-44bd-acc7-d82ff742984c" alt="Software Architecture Diagram" width=600>
</div>

## 📘 Documentation
API Docs: Powered by Doxygen for a deep dive into the codebase.

## 📋 Style Guidelines
Please make sure to pass the **flake8** checks for styling conformity, we use /.precommit-config.yaml file to auto-check files with flake8 linting and good code practices. Commits that do not pass this test can't be pushed.


## 📜 License
Distributed under the MIT License. See LICENSE for more information.
<div align="center">
  Innovate, Integrate, and Elevate your experience with GU Systems.
</div>
