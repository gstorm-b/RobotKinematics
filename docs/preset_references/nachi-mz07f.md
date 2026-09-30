This is Nachi robot MZ07F's preset reference for Kinematics setting.
Pairs of joint point and coordinate get from robot teach pendant, robot original frame coordination.

Joint:
    1  J(21.4862,64.4801,-41.7744,-180.007,22.7216,-158.513)
    2  J(18.6965,68.6763,-39.6996,-158.418,38.2621,-180.401)
    3  J(4.98917,73.5581,-44.4732,-189.95,35.3251,-153.57)
    4  J(-20.077,72.1081,-41.8598,-0.0550513,-30.2747,-20.0276)
    5  J(-17.8857,69.2501,-38.4703,-14.4347,-45.5303,-25.5808)
    6  J(-25.4971,70.7751,-42.0408,44.0796,-38.9396,-79.9948)

Coordinate with tool (center of offset: 44.2, 0, 139.0, 0, 0, 0): X, Y, Z, RZ, RY, RX
    1  X(339.525,133.617,278.473,0.000535733,0.0236197,179.986)
    2  X(339.486,133.658,311.388,1.99842,10.7741,-169.478)
    3  X(339.5,21.8373,319.019,-13.2406,3.7502,172.732)
    4  X(339.503,-124.14,319.977,-0.00232496,0.038313,179.985)
    5  X(339.474,-124.15,319.966,19.152,17.1551,179.99)
    6  X(339.49,-124.109,319.939,14.2266,-16.2692,-159.49)

Note (2026-09-30): although the heading above says "with tool", these coordinates are the
**tool-flange** pose. They are reproduced by the kinematics below only with a zero tool offset;
applying the (44.2, 0, 139.0) mm offset shifts every point by ~145.8 mm (the offset length).
Treat them as flange coordinates. The preset therefore defines only the default (zero) tool.

Joint limits (actual limits of the reference robot; applied to the preset):
J1 (-170.0, 170.0)
J2 (-45.0, 170.0)
J3 (-65.0, 190.0)
J4 (-190.0, 190.0)
J5 (-120.0, 120.0)
J6 (-360.0, 360.0)

---

## Kinematics verification (2026-09-30)

The orientation triple is reported **Z-first** like the MZ04D pendant:
`(yaw about Z, pitch about Y, roll about X)`, i.e. `R = Rz(1st)·Ry(2nd)·Rx(3rd)`.

Standard DH (`Rz(theta) Tz(d) Tx(a) Rx(alpha)`), same structure as MZ04D plus a 50 mm
shoulder offset between the J1 and J2 axes:

|     | theta | d (mm) | alpha (deg) | a (mm) |
|-----|-------|--------|-------------|--------|
| J1  | 0     | 355.0  | +90         | 50     |
| J2  | 0     | 0      | 0           | 330    |
| J3  | 0     | 0      | +90         | 45     |
| J4  | 0     | 340    | -90         | 0      |
| J5  | 0     | 0      | +90         | 0      |
| J6  | 0     | 78     | 0           | 0      |

These nominal lengths reproduce all 6 reference flange poses to **<= 0.065 mm and <= 0.0124 deg**.
A least-squares fit of the six lengths only lowers the residual to <= 0.027 mm while moving each
length by at most 0.24 mm; with 6 points clustered in one region that is treated as overfitting,
so the nominal values are kept. The table is encoded as canonical joint origins in
`src/Presets/NachiMZ07F.cpp` / `presets/Nachi/MZ07F/nachi_mz07f.json` and verified point-by-point
in `tests/integration/NachiMZ07FTests.cpp`.

Because of the J1/J2 shoulder offset, the analytic spherical-wrist IK plugin (which requires
intersecting J1/J2 axes) does not support this model; IK uses the numerical solver.

Posture labels are assumed to follow the same Nachi rules as MZ04D (J1<0 righty / J1>0 lefty,
J3<0 above / J3>0 below, J5<0 flip / J5>0 non-flip); they were not separately measured on MZ07F.

## Collision profiles (2026-09-30)

- `presets/Nachi/MZ07F/nachi_mz07f_mesh_collision.json`: the seven original STL assets. They are
  authored in **meters** (ASCII STL), so every mesh declares `sourceUnits: "m"` and
  `scaleToMeters: 1.0`. The `meshToLink` transforms are derived from the STEP assembly (next
  section); `examples/Robot3DVizualize` renders each STL at `FK(link) * meshToLink`, so the view and
  the mesh collision geometry agree.
- `presets/Nachi/MZ07F/nachi_mz07f_collision.json`: conservative primitives placed in canonical link
  frames from the DH table, sized from STL bounding boxes. They are clear at home, the example
  midpoint, and all six teach-pendant poses; elbow folds below about J3 = -50 deg report a
  conservative collision. They are a fast debug approximation, not an enclosing hull: measured
  against the placed meshes they cover roughly 13 % (J2 upper arm) to 100 % (J6) of each link's
  mesh vertices, comparable to the MZ04D primitive profile.

## meshToLink from the STEP assembly (2026-09-30)

`presets/Nachi/MZ07F/MZ07F.step` is the assembly `MZ07F-01` (millimeters, Y-up) of the seven parts
the STL files were exported from; each STL keeps its STEP part frame (scaled to meters). The
assembly is modeled in the Nachi reference posture (upper arm vertical, forearm horizontal), which
is model joints `q_ref = (0, 90, 0, 0, 0, 0)` deg, the same posture as MZ04D pose 21.

Every component is placed with rotation `diag(-1, 1, -1)` and these translations (mm):

| Part | Assembly translation | Base-frame origin at q_ref | Link      |
|------|----------------------|----------------------------|-----------|
| base | (0, 0, 0)            | (0, 0, 0)                  | base_link |
| j1   | (0, 192, 0)          | (0, 0, 192)                | link_1    |
| j2   | (-50, 355, 0)        | (50, 0, 355)               | link_2    |
| j3   | (-50, 685, 0)        | (50, 0, 685)               | link_3    |
| j4   | (-141, 730, 0)       | (141, 0, 730)              | link_4    |
| j5   | (-390, 730, 0)       | (390, 0, 730)              | link_5    |
| j6   | (-458.5, 730, 0)     | (458.5, 0, 730)            | flange    |

The assembly is mapped into the Z-up base frame (arm along +X at J1 = 0) with
`X_base = -x_asm, Y_base = z_asm, Z_base = y_asm`, which turns every part rotation into `Rx(+90 deg)`
(the same orientation the MZ04D visual corrections use). Then
`meshToLink = inverse(FK(link, q_ref)) * [Rx(90 deg), origin]`.

The placements agree with the DH table independently of the pendant fit: J2 at 355 mm height with
the 50 mm shoulder offset, J3 330 mm above J2, the 45 mm elbow offset, and the wrist center 340 mm
from J3. The J6 part is a 9.5 mm flange disc whose outer face lands exactly on the DH flange frame
(78 mm from the wrist center) plus a ~10 mm pilot boss beyond it, hence `meshToLink` z = -9.5 mm.

Verified in `tests/integration/NachiMZ07FTests.cpp`: FK reproduces the table above, and with the
Coal backend the original meshes are collision-free at q_ref and all six teach-pendant poses while
the folded pose `(0, -45, -65, 0, 0, 0)` drives the wrist into the base/turret. The resulting mesh
placement was visually confirmed by the user in `examples/Robot3DVizualize` on 2026-09-30.
