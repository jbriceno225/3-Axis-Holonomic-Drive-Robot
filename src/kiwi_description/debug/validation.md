# Validation Report: Kiwi_Drive_Full_Assembly_Copy_Copy

## Status: PASS (with warnings)

## Summary

| Metric | Value |
|--------|-------|
| Links | 5 |
| Joints | 4 |
| Assemblies | 17 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 1 |

## Warnings

- Root link renamed: 'Rear_Base_Plate' → 'base_link' (REP 120 convention). Consider renaming the component to 'base_link' in Fusion.

## Kinematic Tree

```
base_link [BAKE] [primitive]
  └─ wheel_1_to_base_link [continuous]
    wheel_1_link_Omni_Wheel [BAKE] [primitive]
  └─ wheel_2_to_base_link [continuous]
    wheel_2_link_Omni_Wheel [BAKE] [primitive]
  └─ wheel_3_to_base_link [continuous]
    wheel_3_link_Omni_Wheel [BAKE] [primitive]
  └─ base_link_to_lidar_link [fixed]
    Lidar_Mounting_Plate [primitive]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `Lidar_Mounting_Plate` | primitive STL | box |
| `base_link` | primitive STL | box |
| `wheel_1_link_Omni_Wheel` | primitive STL | box |
| `wheel_2_link_Omni_Wheel` | primitive STL | box |
| `wheel_3_link_Omni_Wheel` | primitive STL | box |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `base_link` | (0.0, 0.0, 0.0) |
| `wheel_1_link_Omni_Wheel` | (-88.8, 603.2, 368.0) |
| `wheel_2_link_Omni_Wheel` | (129.4, -595.8, 368.0) |
| `wheel_3_link_Omni_Wheel` | (-89.8, 603.1, 368.0) |
