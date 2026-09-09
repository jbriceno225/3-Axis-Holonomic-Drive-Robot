# PCB

KiCad source and fabrication outputs for the Rev A KiwiDrive
module-carrier/control board.

## Layout

- `Rev_A/KiCad_Project/`: schematic, PCB layout, project settings, and local
  footprint-library mapping.
- `Rev_A/Fabrication/Gerbers/`: copper, mask, paste, silkscreen, edge cuts, and
  job file.
- `Rev_A/Fabrication/Drill_Files/`: plated and non-plated drill data.
- `Rev_A/Fabrication/BOM/`: purchasing BOM exported from the project.
- `Libraries/Footprints/`: project-specific footprints for the ESP32 DevKit,
  TB6612FNG carrier, motor/encoder connector, battery input, test point, and
  logo.

Rev A carries an ESP32 DevKit V1, three TB6612FNG driver modules, a
R-78E5.0-1.0 converter module, and motor/encoder and power interconnects. See
[`docs/hardware.md`](../docs/hardware.md) for firmware pin mapping and the
Rev B direction.

## Review before fabrication

The presence of Gerbers is not fabrication approval. Open the KiCad source and
run electrical and design-rule checks. Then verify:

- board outline, layer count, drill tolerances, and manufacturer rules;
- footprint dimensions, pin 1/orientation, connector gender, and keying;
- battery polarity, voltage ratings, motor stall current, copper width,
  grounding, return paths, clearance, and thermal margin;
- converter and driver module part numbers against the actual purchased parts;
- schematic-to-firmware GPIO continuity;
- BOM completeness and substitutions;
- Gerber/drill alignment in an independent viewer.

Initial power-up must use a current-limited supply with motors disconnected.
Rev A does not provide the planned Rev B master switch, branch fuses,
protection/passives, status indicators, or integrated drivers; add external
protection and a physical emergency cutoff during development.

When releasing revised fabrication files, keep editable source and outputs in
the same revision directory and record the KiCad version used. Do not modify
third-party footprint headers or licensing without attribution.
