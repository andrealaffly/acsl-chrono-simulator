# ACSL Chrono Simulator

[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

## 📚 Overview

This package is a C++ implementation of Project Chrono that provides a high fidelity physics engine to perform hardware in-the-loop simulations with the freeware, open-source, PX4 compatible [**ACSL Flightstack Winged**](https://github.com/andrealaffly/ACSL-flightstack-winged). The main purpose of this package is to leverage the simulation environment to test out various **GNC** (Guidance, Navigation and Control) algorithms prior to real-world flights to save on time and to debug the pipeline.

### System Requirements

- **Processor:** Intel
- **RAM:** At least 16 GB
- **GPU:** Nvidia
- **Operating System:** Ubuntu

#### Current Setup

| **Component**    | **Details**                                |
|------------------|--------------------------------------------|
| **CPU**          | Intel 11th Gen i9-11900H                   |
| **GPU**          | Nvidia RTX 3060 (Laptop GPU)               |
| **Operating System** | Ubuntu 20.04                          |
| **RAM**          | 16 GB                                      |

## 💻 How to Use the Repository

### 1. Clone the repository

To clone this repo with all the needed submodules run the command:

```bash
git clone --recurse-submodules https://github.com/girimugundankumar/acsl-chrono-simulator.git
```

Update all the submodules to the stable version

```bash
# Navigate into the simulator directory
cd acsl-chrono-simulator

# Update the submodules
git submodule update --init --recursive
```

### 2. Build the dependencies

After cloning the project and all its dependencies in the previous step, build the dependencies [here](libraries/INSTALL.md).

### Opitional - Solidworks Add-in

After cloning the project and all its dependcies in the previous step, install the optional solidworks plugin by following the instructions [here](/libraries/third-party/chrono-solidworks-installer/SOLIDWORKS_PLUGIN.md).

## 📝 License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](https://github.com/girimugundankumar/acsl-chorno-simulator/blob/main/LICENSE) file for details. The Chrono Project is licensed under the BSD 3-Clause License. See the [LICENSE](https://github.com/projectchrono/chrono/blob/9.0.1/LICENSE) file for more details.



