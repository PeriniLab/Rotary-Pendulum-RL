# Bill of Materials

This directory contains the bill of materials for the hardware design of the Rotary Pendulum.


## Table of Contents


<!-- TOC depthFrom:2 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [Bill of Materials](#bill-of-materials)
    - [Electronics](#electronics)
        - [Microcontroller](#microcontroller)
        - [Motor Driver](#motor-driver)
        - [Power Supply](#power-supply)
        - [Sensors](#sensors)
            - [Encoder](#encoder)
            - [Hall Effect Sensor](#hall-effect-sensor)
    - [Mechanical](#mechanical)
        - [Frame](#frame)
        - [3D Printed Parts](#3d-printed-parts)
        - [Motor](#motor)
        <!-- - [Miscellaneous](#miscellaneous) -->

<!-- /TOC -->

## Electronics

The following table lists the electronic components used in the design. You may use different components, but make sure that they are compatible with the design.

### Microcontroller

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|-----------|
| [ESP32-DevKit](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html) | ESP32 Development Board | 1 | ~€10.00 | [Schematics](https://dl.espressif.com/dl/schematics/esp32_devkitc_v4-sch.pdf) |

### Motor Driver

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [A4988](https://www.pololu.com/product/1182) | Stepper Motor Driver | 1 | ~€3.00 | [Schematics](https://www.pololu.com/file/0J450/a4988_DMOS_microstepping_driver_with_translator.pdf) |

### Power Supply

The motor is powered using a 12V power supply. The ESP32 and other components are powered using a 5V power supply. The 5V power supply is generated using a step-down voltage regulator.

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [LM7805](https://www.sparkfun.com/datasheets/Components/LM7805.pdf) | 5V Voltage Regulator | 1 | ~€0.50 |  |

### Sensors

#### Encoder

An incremental rotary encoder is used to measure the bar angle. The encoder has 600 pulses per revolution and it is powered using 5V.

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [Rotary Encoder](https://www.fruugo.it/600p-r-encoder-di-rotazione-incrementale-magnetoelettrico-5v24v-ab-2fasi-albero-6mm/p-120440516-253131421?language=en) | 600 P/R Incremental Encoder | 1 | ~€20.00 | [Datasheet](datasheets/incremental_rotary_encoder_datasheet.pdf) |

#### Hall Effect Sensor

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [KY-024](https://arduinomodules.info/ky-024-linear-magnetic-hall-module/) | Linear Hall Effect Sensor | 1 | ~€1.00 | [Datasheet](datasheets/linear_magnetic_hall_sensor_datasheet.PDF) |

## Mechanical

### Frame

The frame is made out of an alluminium profile with a cross section of 20x20mm. The length of the profile is around 180mm.

You can use any other material with a different cross section, but make sure that the dimensions are compatible with the design and it is strong enough to sustain the stesses.

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [20x20mm Alluminium Profile](https://www.motedis.com/en/Aluminium-Profile-20x20-I-Typ-slot-5) | 20x20mm Alluminium Profile | 1 | ~€2.00 | [3D Model](https://www.thingiverse.com/thing:6324741) |

### 3D Printed Parts

You can find the 3D models of the parts in the [3d_models/stl](../3d_models/stl/) directory.
The parts were printed using PLA but you can use any other material (try avoiding rubber 😜).

I have also posted the models on Thingiverse and Cults3D:

- [Thingiverse](https://www.thingiverse.com/thing:6377165)
- [Cults3D](https://cults3d.com/en/3d-model/various/rotary-pendulum-rl-open-source-project)

### Motor

| Part Number | Description | Quantity | Price | Details |
|-------------|-------------|----------|-------|---------|
| [NEMA 17 Stepper Motor](https://www.omc-stepperonline.com/nema-17-high-temp-stepper-motor-16ncm-22-7oz-in-extruder-motor-insulation-class-h-180c-17hs08-1004s-h) | 17HS08-1004S | 1 | ~€10.00 | [Datasheet](datasheets/nema17-17HS08-1004S.pdf) |
