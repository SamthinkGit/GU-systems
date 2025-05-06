# <div align="center">GU-Systems</div>

<div align="center">
  <img src="/media/images/poc.png" alt="GU Proof of Concept" width=700>

|       |                                                                                                                                                                        |
| ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Code  | ![GitHub](https://img.shields.io/badge/License-Propietary-blue) ![GitHub](https://img.shields.io/badge/Version-0.0.1-blue) ![GitHub](https://img.shields.io/badge/release-in_process-orange) ![GitHub](https://img.shields.io/badge/docs-Doxygen-white) ![GitHub](https://img.shields.io/badge/Python-3.10-green) ![GitHub](https://img.shields.io/badge/LLM-OpenAI-white) |
| Layers | ![GitHub](https://img.shields.io/badge/execution-pyxcel-royalblue) ![GitHub](https://img.shields.io/badge/cognition-FastReact-royalblue)  ![GitHub](https://img.shields.io/badge/api-FastAP-royalblue) |
| CI/CD | ![GitHub](https://img.shields.io/badge/style-flake8-green) ![GitHub](https://img.shields.io/badge/build-passing-green)                                                 |
| Wiki  | <div align="center"><a href="https://github.com/SamthinkGit/GU-systems/wiki"><strong>📖 Access our Wiki for detailed documentation</strong></a></div>                  |
</div>




## 🚀 Introduction

Welcome to **(codename) GU Systems**, where the future of human-computer interaction unfolds. Imagine your computer uses clicks and keystrokes but now controlled through an AI-powered interface that understands your intent and seamlessly translates it into actions. This is GU Systems - an avant-garde software/hardware suite driven by intelligent AI to elevate your computing to the next level.

## 🎯 Purpose

GU Systems is an approximation to a ECM (Execution Cognition Machine), it deploys multiple layers to interconect Cognition with Execution, thus to fully understand the user, proccess, think, and control a computer as a human-AI. Automation of complex tasks, plannification and AI assistance are the main goals of this repository.

</div>
<div align="center">
  <img src="/media/images/terminal.gif" alt="GU Systems - Terminal example" width=700>
</div>

## 🛠️ Installation
# Installation Guide

## Prerequisites

- It is **recommended** to use a Conda environment for this project. If you choose to do so, install Conda beforehand.
- An OpenAI API key is required. Obtain one from [OpenAI](https://platform.openai.com/signup/) and store it in the `OPENAI_API_KEY` environment variable or enter it when prompted during installation.

## Installation
First, clone the repository to your machine.

```bash
git clone <repository_url>
cd <repository_folder>
```

### Linux

If you are using Linux, run the auto-installation script:

```bash
./installer/install.sh
```

> Note: The linux installation only supports `server` mode. In the current version, action space is only ensured to be stable for Windows installations.

### Windows
If you are using Windows please open the file `./installer/install.bat` with admin permissions.
> You can double click on the file and click on `Run as Administrator`

## Running the Installer
Both options for Windows/Linux will open a streamlit browser where you can install all the dependencies for the ECM.

- For using a local and ready-to-use configuration you can install from `Standard User` tab.
- For updating or installing advanced/experimental features you can use the `Developer` tab.
 
Now you are ready to use the repository!

## 📖 Usage
Using the ECM is as simple as running a python script!
You can use DarkVFR (our best agent so far) to start sending queries to control your computer!

```bash
# For safety reasons this does not affect your computer.
# You can use --host if you are in a safe environment.

python ecm/core/main/main.py --agent darkvfr
```

> **REMEMBER**: If you installed the ECM with conda don't forget to use `conda activate ecm` to init the dependencies!

If you want to have the controller/controlled in different devices you can use the following commands:
```bash
# This is the server (The agent who thinks and reasons)
python ecm/core/main/main.py --agent darkvfr --server
```
```
# This is the client (The device controlled by the agent)
python ecm/core/main/main.py --agent darkvfr --client
```

> Note: If it's the first time connecting two devices, you must append `--autodiscover` to each option to sync the devices

> Note: You can use `--localhost` to allow the same device for both client and server 

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
  <img src="/media/images/layers.png" alt="ECM Layers Diagram" width=450>
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
