# Robot Preset JSON Schema

## Decision

Robot preset files use schema id `robot-kinematics-preset/v1`.

The schema is intentionally close to the internal canonical model:

- links
- ordered joints
- joint origin transforms
- joint axes
- frames
- tools
- posture metadata
- solver metadata
- source references

All solver-facing numeric values are stored in SI units only:

- length: meter
- angle: radian

No JSON preset field performs implicit unit conversion. Human-readable notes can mention vendor units, but the values consumed by the library must already be normalized.

## Required Top-Level Fields

```text
schema
identity
units
topology
links
joints
frames
tools
defaultTool
posture
solver
sources
metadata
```

## Field Contract

### schema

Must equal:

```json
"robot-kinematics-preset/v1"
```

### identity

```json
{
  "vendor": "RobotKinematics",
  "model": "Virtual6DofTestArm",
  "name": "Virtual 6DOF Test Arm",
  "revision": "1.0.0"
}
```

### units

Must be:

```json
{
  "length": "m",
  "angle": "rad"
}
```

### topology

Phase 1 supports:

```json
{
  "type": "serial",
  "dof": 6
}
```

### links

Links are stable IDs. They may be extended later with visual or collision metadata, but phase 1 only requires IDs.

```json
[
  { "id": "base_link" },
  { "id": "link_1" },
  { "id": "flange" }
]
```

### joints

Joints are ordered from base to flange for serial robots.

```json
{
  "id": "J1",
  "type": "revolute",
  "parent": "base_link",
  "child": "link_1",
  "origin": {
    "xyz_m": [0.0, 0.0, 0.0],
    "rpy_rad": [0.0, 0.0, 0.0]
  },
  "axis": [0.0, 0.0, 1.0],
  "limits": {
    "lower": -3.141592653589793,
    "upper": 3.141592653589793,
    "velocity": null,
    "acceleration": null
  },
  "home": 0.0,
  "aliases": ["JT1"]
}
```

For revolute joints, `limits` and `home` are radians. For prismatic joints, they are meters.

### frames

```json
{
  "base": "base_link",
  "flange": "flange",
  "userFrames": [
    {
      "id": "vision_frame",
      "parent": "base_link",
      "transform": {
        "xyz_m": [0.0, 0.0, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0]
      }
    }
  ]
}
```

### tools

```json
[
  {
    "id": "default",
    "name": "Default Tool",
    "flangeToTcp": {
      "xyz_m": [0.0, 0.0, 0.0],
      "rpy_rad": [0.0, 0.0, 0.0]
    }
  }
]
```

### posture

```json
{
  "resolver": "serial_6dof_shoulder_elbow_wrist",
  "labels": {
    "shoulder": { "negative": "lefty", "positive": "righty" },
    "elbow": { "negative": "below", "positive": "above" },
    "wrist": { "negative": "non-flip", "positive": "flip" }
  }
}
```

Preset-specific posture rules may be added under `posture.parameters`.

### solver

```json
{
  "default": "adaptive_damped_least_squares",
  "parameters": {
    "maxIterations": 200,
    "maxSeeds": 32,
    "positionTolerance_m": 0.000001,
    "orientationTolerance_rad": 0.000017453292519943296
  }
}
```

Missing parameters use library defaults.

### sources

```json
[
  {
    "type": "project_fixture",
    "title": "RobotKinematics virtual preset definition",
    "reference": "docs/robot_kinematics_spec.md",
    "appliesTo": ["dimensions", "joint_limits", "posture"]
  }
]
```

Preset implementation should preserve these references so future maintainers know where each robot parameter came from.

The loader stores source references in the canonical `SerialRobotConfig` so generated or built-in presets can be compared without losing provenance.

### metadata

Free-form object for non-solver metadata.

Unknown top-level fields are invalid. Put extension data inside `metadata`.
