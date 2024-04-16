# Comprehensive Guide for Chrono High Fidelity Simulator 
Chorno Simulator used by ACSL. Compile instructions are written by Giri Mugundan Kumar. If you have any question email girimugundan@vt.edu or a.lafflitto@vt.edu for further information.

> **Note:** Start with the [Introduction](#introduction) and follow the [pre-requisites](#pre-requisites) to setup the environment for compiling chorno code.

## Introduction
This guide is made for installing Chrono high fidelity simulator with the following packages. 

1. Core Module
2. FSI Module
3. GPU Module
4. Irrlicht Module
5. Vehicle Module
6. Sensor Module
7. Cascade Module
8. Multicore Module
9. Postprocess Module
10. Python Module
12. Chrono Solidworks Plugin

> **Note:** The installation instruction are provided to install 'all' the mentioned modules. This is done so that every install instance of CHRONO at ACSL is 'identitcal' and all the simulation programs run on different hardware. The modules utilize features that are selected based on the following basic hardware specifications:

> 1. PC running Windows 10/11 (Instructions are validated on Windows 11).
> 2. Any NVIDIA GPU (Laptop or Desktop). 10-series or higher. (Instructions are validated with an NVIDIA RTX 3060 Laptop SKU)
> 3. Intel CPU (Instructions are validated with an 11th Gen Intel i9-11900H @ 4.900GHz)

> For further information, the Laptop used to validate the installation is : ROG Zephyrus M16 GU603HM_GU603HM 1.0

## Pre-Requisites
The pre-requisites for the chorno installation are as follows:

### Git for windows



> **Note:** 

### Microsoft Vistual Studio 2022

Visit this [link](https://visualstudio.microsoft.com/downloads/) and download the 'Community' version of the Visual Studio 2022. Once the installer is done. 
> **Note:** Select the following development in the drop down menu to install your necessary compile tools.

### Python 3.12

Visit this [link](https://www.python.org/downloads/release/python-3122/) and download the 'Windows installer (64-bit)' which is recommened. Go through the process and install it at the default location.

> **Note:** If you already have a Python version installed on your system which is greater than or equal to version 3.6. You can potentially skip this step.

### CMake 

Visit this [link](https://cmake.org/download/) and download the latest CMake version. choose the 'Windows x64 Installer:' under the 'Binary distributions:' window and go through the installation

> **Note:**

### Solidworks 2022 
