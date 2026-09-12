# KiwiDrive — Three-Wheel Holonomic Robot

A work-in-progress mobile robotics platform built around three omni wheels, an ESP32 controller, quadrature encoders, custom mechanical design, and a ROS 2 workspace.

> **Project status:** active development. The repository contains the current design and software artifacts; physical-system validation is in progress.

## What is in this repository

- **Mechanical design:** Fusion 360 assemblies, STEP exports, and printable STL parts in [`CAD/`](CAD/).
- **Electronics:** KiCad Rev A project files, fabrication outputs, and a purchasing BOM in [`PCB/`](PCB/).
- **Embedded control:** ESP32 firmware for three motor channels, encoder feedback, PID velocity control, Bluetooth gamepad control, and a serial-command safety timeout in [`ESP32_Code/`](ESP32_Code/).
- **ROS 2 integration:** Robot-description files, RViz configuration, serial-command bridge code, and vendor dependencies in [`src/`](src/).

## System overview

```text
Operator / ROS 2 command
          |
          v
  Raspberry Pi + ROS 2 workspace
          |
          v
       Serial interface
          |
          v
ESP32 -> motor drivers -> three DC motors
  ^                              |
  +--------- quadrature encoders-+
```

The three-wheel omni-wheel layout is intended to support holonomic translation and rotation. Current software includes manual control and the foundations for command bridging, robot description, and odometry work.

## Repository layout

```text
CAD/          Fusion 360 sources, STEP exports, and 3D-printable parts
PCB/          Rev A KiCad design, Gerbers, drill files, and BOM
ESP32_Code/   ESP32 firmware
src/          ROS 2 description, serial bridge, and third-party dependencies
patches/      Local patch overlays for selected dependencies
```

## Current development focus

- Integrating ESP32 motor control, encoder feedback, and safety behavior.
- Building out the ROS 2 robot description and serial bridge.
- Validating wheel kinematics, odometry, and hardware behavior through staged testing.
- Documenting CAD, PCB, firmware, and ROS 2 interfaces as the system matures.

## Safety

Test with the drive wheels raised during initial firmware and controller checks. Use an accessible physical power cutoff for floor testing; a software timeout is not a substitute for a physical emergency stop.

## Tools

**Mechanical:** Fusion 360, STEP, STL, 3D printing  
**Electronics:** KiCad, ESP32, TB6612FNG motor drivers, quadrature encoders  
**Software:** C/C++, Python, ROS 2, PID control, serial communication

## License

This project is licensed under the [MIT License](LICENSE).
