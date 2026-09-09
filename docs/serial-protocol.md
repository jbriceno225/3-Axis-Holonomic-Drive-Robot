# ESP32 serial protocol

The Raspberry Pi and ESP32 use UTF-8/ASCII, newline-terminated records at
`115200` baud. `\r` is ignored. Commands are at most 96 characters in the
current firmware. The Python implementation lives in
`src/kiwi_serial_bridge/kiwi_serial_bridge/serial_protocol.py`.

> This is the current contract. The checked-in legacy PDF predates the
> `TwistStamped` ROS bridge and must not be used as the command authority.

## Host-to-controller commands

| Record | Meaning |
| --- | --- |
| `MODE,AUTO` | stop, reset control state, and enter autonomous mode |
| `MODE,MANUAL` | stop, reset control state, and enter gamepad mode |
| `V,x,y,rotation` | normalized `[-1, 1]` robot-right, robot-forward, rotation |
| `STOP` | command zero wheel targets |
| `PING` | request liveness acknowledgement |

Example:

```text
MODE,AUTO
V,0.0000,0.2000,0.0000
STOP
```

Velocity commands are accepted only in AUTO and are clamped. The ESP32 stops
after 300 ms without a refreshed command. The ROS bridge sends at 20 Hz and
zeros a ROS command older than 250 ms.

## Controller-to-host records

| Record | Meaning |
| --- | --- |
| `ACK,...` | accepted command or state transition |
| `WARN,...` | nonfatal condition, including autonomous timeout |
| `ERR,...` | rejected or malformed command |
| `MODE,<mode>,SOURCE,<source>` | mode-change diagnostic |
| `ODOM,rpm1,rpm2,rpm3,count1,count2,count3` | wheel telemetry, nominally 20 Hz |

Human-readable controller diagnostics are also emitted. The bridge ignores
unrecognized lines, except malformed `ODOM` records are warned about.

## Coordinate boundary

ROS `TwistStamped` follows REP-103: `linear.x` forward, `linear.y` left, and
`angular.z` counter-clockwise. The bridge maps this to firmware fields:

- protocol `x`: robot right;
- protocol `y`: robot forward;
- protocol `rotation`: configured yaw direction.

Never send SI units in `V`; its fields are normalized fractions. The bridge
scales against `0.30 m/s` and `1.00 rad/s` defaults before formatting.

## Safe terminal test

Disconnect motor power or lift all wheels:

```bash
python3 -m serial.tools.miniterm /dev/kiwi_esp32 115200
```

Send `PING` first and expect `ACK,PONG`. Enter AUTO only with a physical stop
available. Send repeated low values; a single velocity record intentionally
times out after 300 ms. Return to `MODE,MANUAL` before disconnecting.

## Compatibility changes

Protocol changes must update firmware, Python formatter/parser, unit tests,
this document, and the bridge configuration together. Prefer additive,
versioned records; do not silently change field order, units, signs, or
timeouts.
