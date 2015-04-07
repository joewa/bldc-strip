bldc-strip
==========

Firmware for OpenBLDC "Strip" (STM32F103).

## Project description

Bldc-strip is an open-source project aiming to design an ESC (electronic speed controller) for brushless dc motors. The control software is concurrently developed with a motor controller [hardware](https://github.com/joewa/open-bldc-hardware/tree/master/strip/v0.2) and [simulation models](https://github.com/joewa/open-bldc-modelica) which are used to investigate and test the control algorithms before/while they are implemented to the real controller software and hardware. It is designed to provide a better ESC for the control of small multi-rotor aircraft but should also fit other applications.

## Features

- [x] Open-source control software for STM32F103 32bit microcontroller using [ChibiOS](http://www.chibios.org).
- [x] Sensorless 6-step control method for brushless motors.
- [x] Open loop voltage control to provide (almost) linear "brushed-motor-like" characteristics which is often requested by control systems engineers.
- [x] High frequency PWM from 40 to 200 kHz enabling proper operation of some very small and fast motors.
- [x] Dedicated control methods for fast, slow and very slow to zero rotation speed.
- [x] Seamless 4-quadrant operation with regenerative braking.
- [x] Smooth change of rotation direction.
- [ ] Adequate protection functions:
  - [ ] limitation of motor current
  - [ ] limitation of input current
  - [ ] seperate limit for regenerative current
  - [ ] limitation of reference duty cycle command that would result in a current limitation
- [ ] Software watchdog
- [x] Command/control interface via UART
- [ ] Command/control interface via CAN-bus

## Dependencies and references

Designed and tested with the [OpenBLDC "Strip" V0.2 ESC](https://github.com/joewa/open-bldc-hardware/tree/master/strip/v0.2).

Using [ChibiOS Real-Time Operating System](http://www.chibios.org).

Multi-physics, open-source [Modelica simulation models](https://github.com/joewa/open-bldc-modelica) are designed to investigate appropriate motor control methods. In the future, they may be merged with this repository.

Overview of sensorless 6-step control methods.

## Status and contribution

This is a part-time development project and is not dedicated for productive usage. Any contributions preferably [Pull Requests](https://github.com/joewa/bldc-strip/pulls) are always welcome.

You may report any issues or share ideas by using the [Issues](https://github.com/joewa/bldc-strip/issues) button.

## License

The project is released under the GNU GENERAL PUBLIC LICENSE - Version 3.

