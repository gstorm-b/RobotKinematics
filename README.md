# RobotKinematics

RobotKinematics is a planned C++/Eigen backend library for industrial robot kinematics.

The library will support forward kinematics, inverse kinematics, joint-limit validation, user frames, tool frames, custom robot presets, and reusable solver interfaces. It has no UI.

## Current Status

The project is in early implementation. Phase 0 has scaffolded the Qt/qmake library project and Qt Test executable. Phase 1 core types are implemented: unit helpers, `Pose`, canonical serial robot model value types, `KinematicsStatus`, and serial model validation. Phase 2 is implemented and MSVC-verified: joint vectors, joint-limit validation, frame/tool registries, and forward kinematics. Phase 3 has IK API types, an adaptive damped least-squares numerical solver, and `SerialRobotKinematics::solve/solveAll` orchestration with frame/tool resolution. Phase 4 has generic posture metadata, a metadata-gated serial 6DOF posture resolver, and posture-aware IK scoring/rejection. Phase 5 has custom config building, preset JSON loading, and the `Virtual6DofTestArm` JSON plus C++ fallback. Phase 7 has standard DH import to canonical config and URDF-like export/import adapters.

The first engineering milestone is the base serial 6DOF implementation using a project-owned virtual preset named `Virtual6DofTestArm`. Real robot presets for Kawasaki RS007N and Nachi MZ04D are validation targets after verified source data is provided.

## Start Here

Read these files in order:

1. [Project Spec](docs/robot_kinematics_spec.md)
2. [Implementation Plan](docs/robot_kinematics_implementation_plan.md)
3. [Preset JSON Schema](docs/robot_preset_json_schema.md)
4. [Developer Onboarding](docs/developer_onboarding.md)
5. [Agent Instructions](AGENTS.md), if you are an AI/code agent working on this repo

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

Real Kawasaki RS007N and Nachi MZ04D presets are still blocked until verified dimensions, joint limits, posture rules, and source references are provided.

Canonical serial configs may contain fixed joints; `dof` counts movable joints only.

## Build And Test

From the repository root:

```powershell
qmake RobotKinematics.pro
nmake
.\tests\RobotKinematicsTests.exe
```

For MinGW, use `mingw32-make` instead of `nmake`.

## Non-Goals For The First Milestone

- No UI.
- No physical robot accuracy claim.
- No collision checking.
- No path planning.
- No SCARA, delta, parallel, 4DOF, or 5DOF implementation.
- No exhaustive guarantee for numerical `solveAll`.
- No real Kawasaki/Nachi preset implementation until source data is provided.
