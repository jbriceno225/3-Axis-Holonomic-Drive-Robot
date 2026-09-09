# CAD

Mechanical source and exports for the three-wheel KiwiDrive platform.

## Contents

- `Assembly/Fusion_360/`: editable Fusion 360 assembly archive (`.f3z`).
- `Assembly/STEP/`: neutral full-assembly exchange model.
- `Printed_Parts/Fusion_360/`: editable part sources (`.f3d`).
- `Printed_Parts/STEP/`: neutral part exports.
- `Printed_Parts/STL/`: fabrication meshes for slicing.
- `Reference_Models/`: vendor/component geometry used for fit checks; these
  models are references, not project-manufactured parts.

The model includes frame plates, 2020-extrusion spacers, motor mounts/hubs,
battery mount, PCB enclosure, LiDAR plate, motors, omni wheels, coupling, and
electronics references.

## Working with the files

1. Modify the Fusion source where available.
2. Check assembly joints, interference, wheel contact plane, cable clearance,
   fastener access, and center of mass.
3. Export STEP for review and STL only for printable bodies.
4. Slice STLs using material- and printer-appropriate orientation, supports,
   walls, and infill. No universal print settings are validated here.
5. Record the CAD tool/version and regenerate affected ROS meshes when geometry
   changes.

Dimensions inferred from exported models are not a substitute for measuring
the assembled robot. The ROS stack currently uses a `0.050 m` wheel radius,
`0.2921 m` center-to-wheel distance, and provisional LiDAR pose; reconcile
those values after mechanical changes.

Do not overwrite reference models with modified parts. Respect any
third-party licensing attached to imported component geometry.
