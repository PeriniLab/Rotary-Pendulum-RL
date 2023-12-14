# Rotary-Pendulum-RL - Open Source Project

This is an open source project aimed at introducing people to **robotics**, from the hardware and embedded systems implementation to the software and control algorithms. The goal is to control a real life **rotary pendulum** to make it perform a swing-up and balance task.

The project is divided into four parts: **hardware**, **firmware**, **simulation** and **control**. The hardware part is responsible for the mechanical design and the electronic components. The firmware part is responsible for the embedded systems implementation. The simulation part is responsible for the simulation of the rotary pendulum in a virtual environment. The control part is responsible for the implementation of the control algorithms.

## Folder Structure

```bash
├─── hardware
│   ├─── 3d_models
│   ├─── BOM
│   ├─── schematics
│   └─── instructions
│
├─── firmware
│   ├─── arduino
│   └─── platformio
│
├─── simulation
│   ├─── urdf
│   ├─── pybullet
│   └─── gazebo
│
└─── control
    ├─── optimal_control
    └─── reinforcement_learning
```
### 📂[hardware](hardware)

This folder contains the mechanical design and the electronic schematics as well as the instructions on how to build the rotary pendulum.
It will guide you through the process of 3D printing the parts, choosing the right mechanical and electronic components and assembling the rotary pendulum.

### 📂[firmware](firmware)

This folder contains the microcontroller code that will run on the embedded systems of the rotary pendulum.
It will guide you through the process of setting up the development environment, compiling and uploading the code to the microcontroller.

### 📂[simulation](simulation)

This folder contains the simulation code that will run on the computer. It will simulate the rotary pendulum in a virtual environment, such as the [PyBullet](https://pybullet.org/wordpress/) or [Gazebo](http://gazebosim.org/) simulators.

### 📂[control](control)

This folder contains the control code that will run on the computer. It will implement the control algorithms that will be used to control the rotary pendulum, from model-free to model-based control algorithms.

## Getting Started

### Prerequisites

* [Python 3.6+](https://www.python.org/downloads/)
* [Git](https://git-scm.com/downloads)
* [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/install/ide?install=vscode)
* [Visual Studio Code](https://code.visualstudio.com/download) or IDE of your choice

### Installation

#### Windows

If you plan to simulate the rotary pendulum on your PC, for example using PyBullet, you will need to install [Build Tools for Visual Studio 2022](https://aka.ms/vs/17/release/vs_BuildTools.exe). Make sure to select the **C++ build tools** option.

You can as well install them in one go using [winget](https://docs.microsoft.com/en-us/windows/package-manager/winget/) in the command line:
```bash
# Windows 10 SDK
winget install Microsoft.VisualStudio.2022.BuildTools --force --override "--wait --passive --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows10SDK"

# Windows 11 SDK
winget install Microsoft.VisualStudio.2022.BuildTools --force --override "--wait --passive --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows11SDK.22000"

```
#### Clone the repository

```bash
git clone https://github.com/PeriniM/Rotary-Pendulum-RL.git
cd Rotary-Pendulum-RL
```
#### Create a virtual environment

It is recommended to create a virtual environment to install the dependencies in order to avoid conflicts with other projects.

```bash
python -m venv venv
# python3 -m venv venv
```
#### Activate the virtual environment

All the commands must be executed in the virtual environment. If you are not familiar with virtual environments, please read the [Python Virtual Environments: A Primer](https://realpython.com/python-virtual-environments-a-primer/) article.

To activate the virtual environment, run the following command:
```bash
# Windows
.\venv\Scripts\activate
# Linux
source venv/bin/activate
```

#### Install the dependencies

In the **requirements.txt** file you will find all the dependencies needed to run the code. To install them, run the following command:

```bash
pip install -r requirements.txt
# pip3 install -r requirements.txt
```

## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you want to contribute to this project, please read the [CONTRIBUTING.md](CONTRIBUTING.md) file.

## License

Distributed under the **Apache License 2.0**. See [LICENSE](LICENSE) for more information.
