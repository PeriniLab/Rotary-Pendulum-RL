Installation
------------

In the following sections I will guide you through the installation process of the required components
for this project. From the python dependencies (for running the simulation and controlling the robot)
to the set-up of the microcontroller firmware development.

If you would like to jump straight to the building and assembling of the robot and then come back later,
you can do so by skipping to the :ref:`bill of materials <bom>` section.

Prerequisites
^^^^^^^^^^^^^

- `Python 3.8+ <https://www.python.org/downloads/>`_
- `Git <https://git-scm.com/downloads>`_
- `Arduino IDE <https://www.arduino.cc/en/software>`_ or `PlatformIO <https://platformio.org/install/ide?install=vscode>`_
- `Visual Studio Code <https://code.visualstudio.com/download>`_ or IDE of your choice

External dependencies
^^^^^^^^^^^^^^^^^^^^^

Windows
+++++++

If you plan to simulate the rotary pendulum on your PC, for example using PyBullet, you will need to install `Build Tools for Visual Studio 2022 <https://aka.ms/vs/17/release/vs_BuildTools.exe>`_. Make sure to select the **C++ build tools** option.

You can as well install them in one go using `winget <https://docs.microsoft.com/en-us/windows/package-manager/winget/>`_ in the command line:

.. code-block:: bash

   # Windows 10 SDK
   winget install Microsoft.VisualStudio.2022.BuildTools --force --override "--wait --passive --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows10SDK"

   # Windows 11 SDK
   winget install Microsoft.VisualStudio.2022.BuildTools --force --override "--wait --passive --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 --add Microsoft.VisualStudio.Component.Windows11SDK.22000"

Linux
++++++

You don't need to install any external dependencies.

Clone the repository
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git clone https://github.com/PeriniM/Rotary-Pendulum-RL.git
   cd Rotary-Pendulum-RL

Create a virtual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is recommended to create a virtual environment to install the dependencies in order to avoid conflicts with other projects.

.. code-block:: bash

   python -m venv venv
   # python3 -m venv venv

Activate the virtual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All the commands must be executed in the virtual environment. If you are not familiar with virtual environments, please read the `Python Virtual Environments: A Primer <https://realpython.com/python-virtual-environments-a-primer/>`_ article.

To activate the virtual environment, run the following command:

.. code-block:: bash

   # Windows
   .\venv\Scripts\activate
   # Linux
   source venv/bin/activate

Install the dependencies
^^^^^^^^^^^^^^^^^^^^^^^^

In the **requirements.txt** file you will find all the dependencies needed to run the code. To install them, run the following command:

.. code-block:: bash

   pip install -r requirements.txt
   # pip3 install -r requirements.txt

If you plan to implement some reinforcement learning algorithms, especially using `Stable Baselines 3 <https://stable-baselines.readthedocs.io/en/master/>`_, you will need to install the Pytorch dependencies.
Go to the `Pytorch website <https://pytorch.org/>`_, fill the installation wizard and run the command that will be provided to you. In this way you can opt to use the GPU or not.
An example of the command is the following:

.. code-block:: bash

   pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

Test the installation
^^^^^^^^^^^^^^^^^^^^^

- Let's test if the pybullet installation was successful. Run the following command:

    .. code-block:: bash

       python ./simulation/pybullet/main.py
       # python3 ./simulation/pybullet/main.py

- Let's test if the modules works. Run the following command:

    .. code-block:: bash

       python -m control.reinforcement_learning.src.main
       # or
       python -m control.pid.src.main