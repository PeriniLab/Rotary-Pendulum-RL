.. _firmware_setup:

Set-up
------

In order to be able to develop the microcontroller firmware and flash it to the board, you will need to install some dependencies.

USB-to-serial Driver
^^^^^^^^^^^^^^^^^^^^

When using the ESP32 Devkit board, you will need to install the USB-to-serial driver for the CP210x chip.
You can find the driver for your operating system on the `Silicon Labs website <https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads>`_.
In this way, you will be able to flash the firmware to the board and see the debug messages.

.. note::
    If you are using a different board, you will need to install the USB-to-serial driver for the chip on your board.
    Another common chip is the CH340x.

PlatformIO
^^^^^^^^^^

The firmware is developed using PlatformIO, which is an open source ecosystem for IoT development.
It supports more than 600 development boards and 20 development platforms, including the ESP32 Devkit-Doit-V1.

You can install PlatformIO using the `official documentation <https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode>`_,
which basically consists in installing the PlatformIO extension for Visual Studio Code.

.. note::
    If you want to get started with an already configured project, you can import the existing pendulum firmware which can be found in the 
    `Github repo <https://github.com/PeriniM/Rotary-Pendulum-RL/tree/main/firmware>`_.