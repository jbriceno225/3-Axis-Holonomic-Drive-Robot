# ESP32 firmware

`Kiwi_Drive_Kin_ROS2_Velocity_PID_Bluepad32.ino` is the current Rev A
controller sketch. It provides:

- Bluepad32 gamepad manual translation and rotation;
- three-wheel Kiwi inverse kinematics;
- x4 quadrature encoder counting;
- 50 Hz feed-forward plus PID wheel-velocity control;
- 20 Hz wheel RPM/count telemetry;
- manual/auto mode switching, latched gamepad software stop, and a 300 ms
  autonomous serial watchdog;
- PWM API compatibility branches for Arduino ESP32 core 2.x and 3.x.

See [`docs/serial-protocol.md`](../docs/serial-protocol.md) for the wire
contract and [`docs/calibration.md`](../docs/calibration.md) before tuning.

## Build dependencies

| Dependency | Required/current information |
| --- | --- |
| Board | ESP32 DevKit V1-compatible module |
| Arduino IDE or CLI | Exact tested version needs owner confirmation |
| ESP32 Arduino core | Source supports major 2 and 3 APIs; exact tested version needs owner confirmation |
| Bluepad32 | Required (`Bluepad32.h`); exact tested version/board package needs owner confirmation |
| USB serial | `115200` baud |

For a reproducible release, the owner should record exact Arduino IDE/CLI,
ESP32 core, Bluepad32, board definition, partition scheme, compiler, and upload
settings. Until then, do not describe a particular combination as verified.

## Arduino IDE build

1. Install ESP32 board support and Bluepad32 following Bluepad32's current
   ESP32 installation instructions.
2. Select the exact ESP32 DevKit variant installed on Rev A.
3. Open the `.ino`, compile, and save the complete build/version output.
4. Disconnect motor power, upload over USB, and open Serial Monitor at
   `115200`.
5. Verify the startup pin report against the PCB and actual wiring.

## Pin map

| Motor | PWM | IN1 | IN2 | Encoder A | Encoder B |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 25 | 26 | 27 | 34 | 35 |
| 2 | 13 | 12 | 14 | 36 | 39 |
| 3 | 32 | 33 | 23 | 16 | 17 |

Shared TB6612 standby is GPIO 19. GPIO 34/35/36/39 have no internal pulls.

## Controls and first test

- Left stick: translate.
- Right stick X: rotate.
- Start: toggle MANUAL/AUTO.
- A: latch software stop; B: clear it.

With traction power disconnected, confirm startup and `PING`/`ACK,PONG`.
Then lift all wheels and test one low command at a time. Verify encoder signs,
motor directions, STOP, gamepad disconnect behavior, and autonomous timeout.
A software stop is not a physical emergency stop.

Calibration constants in the sketch are starting values. Preserve a copy of
raw serial logs and test conditions whenever changing them.
