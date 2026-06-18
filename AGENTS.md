# Agent Instructions: RobotKinematics

This file is for AI/code agents implementing work in this repository.

## Required Context Load

Before editing code or docs, read these files:

1. `README.md`
2. `docs/robot_kinematics_spec.md`
3. `docs/robot_kinematics_implementation_plan.md`
4. `docs/robot_preset_json_schema.md`
5. `docs/developer_onboarding.md`

Do not infer missing project decisions from general robotics conventions when the spec already defines a decision.

## Current Project State

The project is in early implementation. Phase 0 and Phase 1 are implemented: the repository has a Qt/qmake static library scaffold, Qt Test runner, unit helpers, `Pose`, canonical serial robot model value types, `KinematicsStatus`, and serial model validation.

The next implementation target is Phase 2: joint vectors, joint limits, frame/tool registry behavior, and FK. Kawasaki RS007N and Nachi MZ04D must wait until the user provides verified dimensions, joint limits, and posture rules.

## Implementation Order

Follow `docs/robot_kinematics_implementation_plan.md` in dependency order:

1. Phase 0: Qt/qmake project foundation.
2. Phase 1: units, pose, canonical model, validation.
3. Phase 2: joint vectors, joint limits, frames, tools, FK.
4. Phase 3: IK API and numerical IK.
5. Phase 4: posture and solution ranking.
6. Phase 5: preset JSON loader and `Virtual6DofTestArm`.
7. Phase 6: real presets only after source data is available.

Do not start adapters, analytic IK, or real robot presets before the base milestone is working unless the user explicitly changes scope.

## Hard Invariants

- Core calculations use meter and radian.
- No implicit unit conversion.
- Public helper names must include units when accepting raw numeric values.
- Core pose representation is `Pose`, backed by `Eigen::Isometry3d`.
- RPY/Euler is an IO/helper format only.
- Canonical model is URDF-like and solver-facing.
- URDF and DH are adapters, not the canonical source of truth.
- Numerical `solveAll` returns solutions found by the selected solver, not mathematically exhaustive solutions.
- Real robot preset source references must be preserved.
- Preset data must not be mixed into solver logic.

## Required Status Enum

Use the status contract from the spec:

```cpp
enum class KinematicsStatus {
    Ok,
    InvalidRobotConfig,
    InvalidRequest,
    FrameNotFound,
    ToolNotFound,
    JointDimensionMismatch,
    JointLimitViolation,
    TargetUnreachable,
    Singularity,
    MaxIterationsReached,
    NoConvergedSolution,
    PostureConstraintUnsatisfied,
    UnsupportedSolver,
    NumericalError
};
```

## Testing Expectations

Use Qt Test.

Every implementation task must include relevant tests. At minimum:

- Unit conversion helpers.
- Pose construction and transform composition.
- Model validation.
- Joint limit validation.
- FK transform chaining.
- Frame/tool composition.
- IK API/status behavior.
- Numerical IK residuals on normal non-singular fixtures.
- Solution ranking.
- Posture resolver behavior.
- Preset JSON loading and C++ fallback equivalence.

Accuracy targets for normal non-singular fixtures:

- Position residual: `<= 1e-6 m`.
- Orientation residual: `<= 1.7453292519943296e-5 rad` (`0.001 degree`).

Do not apply those tolerances to singularities, joint-limit boundaries, or intentionally unreachable targets unless the test is specifically about failure handling.

## Agent Workflow

For each task:

1. Read the relevant spec and plan section.
2. Identify the smallest task slice.
3. Add or update tests first when practical.
4. Implement only what the task requires.
5. Run the narrowest relevant tests.
6. Run the full available test command before handing off if the project can build.
7. Update docs if behavior, API, defaults, schema, or scope changes.

Prefer small, reviewable changes. Do not bundle unrelated phases together.

## Ask Before Changing

Ask the user before:

- Changing unit conventions.
- Changing public API names from the spec.
- Adding dependencies beyond Eigen, Qt, and the selected test framework.
- Claiming physical robot accuracy.
- Implementing SCARA, delta, parallel, 4DOF, or 5DOF support.
- Making URDF the canonical model.
- Moving analytic IK into the first milestone.
- Implementing Kawasaki/Nachi data without source references.

## Good First Tasks

If starting from the current implementation, continue with Phase 2 from the plan. If no code exists in another checkout, start with:

1. Scaffold `RobotKinematics.pro` and test target.
2. Add `include/`, `src/`, `tests/`, and `presets/` directories.
3. Implement `Units`.
4. Implement `Pose`.
5. Implement canonical model value types.

## Handoff Notes

When handing off work, include:

- Task number from the implementation plan.
- Files changed.
- Tests run.
- Known limitations.
- Any doc updates made.
- Any decisions still needing user input.
