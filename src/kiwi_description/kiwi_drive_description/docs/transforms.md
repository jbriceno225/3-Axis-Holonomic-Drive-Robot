# Transformation Matrices - Kiwi_Drive_Full_Assembly_Copy_Copy

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | wheel_1_link_Omni_Wheel |
| $L_{2}$ | wheel_2_link_Omni_Wheel |
| $L_{3}$ | wheel_3_link_Omni_Wheel |
| $L_{4}$ | Lidar_Mounting_Plate |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | wheel_1_to_base_link | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | wheel_2_to_base_link | continuous (rad) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | wheel_3_to_base_link | continuous (rad) | $L_{0}$ | $L_{3}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [continuous] wheel_1_to_base_link (q1)
  |   L1: wheel_1_link_Omni_Wheel
  |-- [continuous] wheel_2_to_base_link (q2)
  |   L2: wheel_2_link_Omni_Wheel
  |-- [continuous] wheel_3_to_base_link (q3)
  |   L3: wheel_3_link_Omni_Wheel
  +-- [fixed] base_link_to_lidar_link
      L4: Lidar_Mounting_Plate
```

## Transforms

## wheel_1_to_base_link

$L_{0}$ **base_link** -> $L_{1}$ **wheel_1_link_Omni_Wheel** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.269094, -0.15514, -0.028295) m
- **origin rpy**: (-1.588336, 0.031105, 1.057255) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.491027 & 0 & -0.871144 & 0.269094 \\
0.870589 & -0.035701 & 0.490714 & -0.15514 \\
-0.0311 & -0.999363 & -0.01753 & -0.028295 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wheel_2_to_base_link

$L_{0}$ **base_link** -> $L_{2}$ **wheel_2_link_Omni_Wheel** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (-0.000532, 0.310563, -0.028788) m
- **origin rpy**: (1.574419, -0.000039, 0.010768) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.999942 & 0 & 0.010768 & -0.000532 \\
0.010768 & -0.003623 & -0.999935 & 0.310563 \\
0.000039 & 0.999993 & -0.003623 & -0.028788 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## wheel_3_to_base_link

$L_{0}$ **base_link** -> $L_{3}$ **wheel_3_link_Omni_Wheel** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (-0.269016, -0.155795, -0.029665) m
- **origin rpy**: (-1.549628, 0.035717, -1.035628) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.50966 & 0 & 0.860376 & -0.269016 \\
-0.859634 & 0.041504 & 0.509221 & -0.155795 \\
-0.035709 & -0.999138 & 0.021153 & -0.029665 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## base_link_to_lidar_link

$L_{0}$ **base_link** -> $L_{4}$ **Lidar_Mounting_Plate** (fixed)

- **origin xyz**: (-0.000151, -0.115009, 0.024945) m
- **origin rpy**: (0, 0, 0) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
1 & 0 & 0 & -0.000151 \\
0 & 1 & 0 & -0.115009 \\
0 & 0 & 1 & 0.024945 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

