# Security and safety reporting

KiwiDrive controls physical motors. Treat unexpected motion, watchdog bypass,
unsafe power behavior, remotely triggerable movement, and exposed secrets as
security/safety issues.

## Immediate response

If a defect could cause motion or electrical danger:

1. Use the physical power cutoff.
2. Disconnect traction power and stop testing.
3. Preserve logs and the exact commit/configuration.
4. Do not reproduce around people, pets, stairs, traffic, or property.

Software `STOP`, the gamepad A-button, watchdogs, and Ctrl-C are defense
layers, not substitutes for a hard power disconnect.

## Reporting

Prefer GitHub private vulnerability reporting from this repository's
**Security** tab when enabled. If it is unavailable, contact the maintainer at
`jbriceno225@gmail.com` with subject `KiwiDrive security report`. For
non-sensitive safety defects, use the bug template.

Include:

- affected commit, package, firmware, and hardware revision;
- impact and realistic preconditions;
- minimal reproduction performed in a safe setup;
- logs or measurements with secrets/device identifiers removed;
- suggested mitigation, if known.

Do not publish exploit details, credentials, personal identifiers, or
instructions that create immediate physical risk before the maintainer has had
an opportunity to respond. No response-time SLA is currently promised.

## Scope

First-party ROS packages, ESP32 firmware, PCB/CAD interfaces, scripts, and
documentation are in scope. Third-party submodules should also be reported to
their upstream maintainers under their policies; tell this project when an
upstream issue affects the pinned integration.
