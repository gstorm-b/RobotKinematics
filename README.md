# RobotKinematics

RobotKinematics is a planned C++/Eigen backend library for industrial robot kinematics.

The library will support forward kinematics, inverse kinematics, joint-limit validation, user frames, tool frames, custom robot presets, reusable solver interfaces, primitive self-collision checks, and an accurate mesh collision backend. It has no UI.

## Current Status

The base serial 6DOF library milestone is implemented. The codebase includes the Qt/qmake static library, Qt Test runner, core units and `Pose`, canonical serial robot model, model validation, joint-limit validation, frame/tool registries, FK, numerical IK, posture-aware solution selection, JSON preset loading, `Virtual6DofTestArm`, standard DH import, URDF-like import/export, and a hybrid analytic IK plugin for supported spherical-wrist 6R robots.

Primitive self-collision detection is implemented as a fast approximate/debug path. Phase 10 backend-neutral mesh scaffolding is now in place: mesh profile types/loaders, STL mesh normalization, a default no-backend `UnsupportedSolver` path, and an optional Coal adapter behind qmake flags are implemented. Synthetic Coal backend tests pass; the next mesh-collision task is authoring the real Nachi MZ04D STL mesh profile. See [docs/mesh_collision_backend_plan.md](docs/mesh_collision_backend_plan.md) and [docs/mesh_collision_backend_spike.md](docs/mesh_collision_backend_spike.md).

Real preset status:

- `NachiMZ04D` is implemented from teach-pendant reference data in [docs/preset_references/nachi-mz04d.md](docs/preset_references/nachi-mz04d.md), with JSON and C++ fallback presets.
- `KawasakiRS007N` is not implemented yet. Task 6.1 remains blocked until verified dimensions, joint limits, posture rules, and source references are provided. See [docs/preset_references/kawasaki-rs007n.md](docs/preset_references/kawasaki-rs007n.md).

## Start Here

Read these files in order:

1. [Project Spec](docs/robot_kinematics_spec.md)
2. [Implementation Plan](docs/robot_kinematics_implementation_plan.md)
3. [Preset JSON Schema](docs/robot_preset_json_schema.md)
4. [Collision Detection Plan](docs/collision_detection_plan.md), if you are working on primitive collision
5. [Mesh Collision Backend Plan](docs/mesh_collision_backend_plan.md), if you are working on accurate mesh collision
6. [Developer Onboarding](docs/developer_onboarding.md)
7. [Agent Instructions](AGENTS.md), if you are an AI/code agent working on this repo

If you want to **use RobotKinematics as a library** (rather than work on it), see the
[Developer Guide](docs/developer-guide/README.md) — building/linking, usage examples, an API
reference, conventions/gotchas, and architecture decision records.

## Core Decisions

- Language: C++.
- Math library: Eigen.
- Build system: Qt 6 with qmake.
- Test framework: Qt Test.
- Primary compiler: MSVC.
- Compatibility compiler: MinGW.
- Core units: meter and radian.
- Pose representation: wrapper around `Eigen::Isometry3d`.
- Canonical robot model: URDF-like links, joints, joint origin transforms, and joint axes.
- Preset format: JSON schema `robot-kinematics-preset/v1`.
- First preset: `Virtual6DofTestArm`.
- First numerical IK method: adaptive damped least squares.
- Collision detection direction: primitive fallback/debug plus backend-neutral accurate mesh scaffolding, with Coal available as an optional compiled backend.

## First Milestone

The first milestone is complete when:

- The Qt/qmake library builds.
- Core model, units, pose, and validation exist.
- FK works for serial 6DOF configs.
- Numerical IK works for normal non-singular serial 6DOF fixtures.
- `solve` and `solveAll` exist with structured request/result types.
- Frames, tools, joint limits, and posture metadata are tested.
- `Virtual6DofTestArm` exists as JSON and built-in C++ fallback.

Current IK limitation: numerical `solveAll` returns found solutions from deterministic seeds, not a mathematically exhaustive branch set. IK reference user frames must currently be fixed to the base link.

Kawasaki RS007N remains blocked until verified dimensions, joint limits, posture rules, and source references are provided. Nachi MZ04D is implemented, with documented caveats around posture-source confidence.

Canonical serial configs may contain fixed joints; `dof` counts movable joints only.

## Build And Test

From the repository root:

```powershell
scripts\build_msvc.bat
scripts\test_msvc.bat
```

For a clean MSVC rebuild plus tests:

```powershell
scripts\rebuild_msvc.bat
```

For MinGW compatibility:

```powershell
scripts\build_mingw.bat
scripts\test_mingw.bat
```

## Non-Goals For The First Milestone

- No UI.
- No physical robot accuracy claim.
- No collision checking in the first milestone. Primitive self-collision is planned as a later extension.
- No path planning.
- No SCARA, delta, parallel, 4DOF, or 5DOF implementation.
- No exhaustive guarantee for numerical `solveAll`.
- No Kawasaki RS007N preset implementation until source data is provided.
