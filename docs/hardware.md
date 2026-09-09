# Hardware and revisions

## System

KiwiDrive uses three omni-wheel modules spaced 120 degrees apart. A Raspberry
Pi 5 runs ROS 2 and communicates over USB serial with an ESP32 DevKit V1. The
ESP32 reads three quadrature encoders and commands three TB6612FNG driver
modules. An LD19 provides planar range scans.

Current geometry used by software:

- wheel radius: `0.050 m`
- center-to-wheel distance: `0.2921 m`
- approximate chassis envelope: `0.50 m`
- provisional `base_link -> base_laser`: `(0.114, 0, 0.10977) m`, yaw
  `-pi/2`

The wheel values are the current measured/configured starting point, not a
completed loaded calibration. The footprint and LiDAR pose must be physically
remeasured on the final assembly.

## Rev A PCB

`PCB/Rev_A/` contains the editable KiCad project and released fabrication
outputs. Rev A is a module-carrier/interconnect board, not the protection-rich
integrated controller planned for Rev B. Its purchasing BOM identifies:

- one ESP32 DevKit V1;
- three TB6612FNG driver modules;
- one R-78E5.0-1.0 5 V converter module;
- battery, motor/encoder, and header connections;
- 29 test points.

Gerbers and a BOM being present does not certify assembly, current capacity,
or electrical safety. Review the KiCad source, module datasheets, polarity,
clearance, and net continuity before ordering or powering a board.

## ESP32 pin mapping

The current firmware defines this mapping:

| Function | GPIO |
| --- | ---: |
| Shared driver standby | 19 |
| Motor 1 PWM / IN1 / IN2 | 25 / 26 / 27 |
| Motor 1 encoder A / B | 34 / 35 |
| Motor 2 PWM / IN1 / IN2 | 13 / 12 / 14 |
| Motor 2 encoder A / B | 36 / 39 |
| Motor 3 PWM / IN1 / IN2 | 32 / 33 / 23 |
| Motor 3 encoder A / B | 16 / 17 |

GPIO 34, 35, 36, and 39 have no internal pull resistors. Verify that the
encoder outputs or external circuitry provide valid logic levels. GPIO 12 is
also a boot-strapping pin on common ESP32 modules; verify the attached driver
does not force an invalid boot state.

Treat the firmware, schematic, PCB layout, and actual wiring as four items
that must agree. Continuity-test every mapping before installing motors.

## Power and test safety

- Work with traction power disconnected during continuity checks.
- Use a current-limited bench supply for initial power-up.
- Verify logic and motor voltage limits against the exact modules fitted.
- Lift wheels for first motion and keep loose wiring clear.
- Install a physical, power-cutting emergency stop. The gamepad A-button,
  serial `STOP`, watchdogs, and Ctrl-C are software controls, not a hard stop.
- Never perform autonomous tests near stairs, traffic, people, or pets.

## Rev B requirements

Rev B should replace carrier-board compromises with a reviewed electrical
architecture:

- accessible master switch and hard emergency power cutoff;
- correctly sized battery and branch fuses;
- reverse-polarity, transient, and over-current protection as appropriate;
- bulk capacitors, local IC bypass capacitors, and explicit pull
  resistors/passives;
- power-good, mode, fault, and driver-status indicators;
- integrated motor drivers selected for stall current and thermal margin;
- keyed connectors, separated high-current returns, test points, and clear
  silkscreen;
- documented stack-up, design-rule checks, current calculations, thermal
  analysis, BOM alternates, and bring-up procedure.

Do not fabricate Rev B until schematic review, layout review, and safety
requirements are closed.
