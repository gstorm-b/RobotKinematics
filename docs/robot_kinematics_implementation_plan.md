# Implementation Plan: RobotKinematics

## Overview

Implement RobotKinematics as a reusable C++/Eigen backend library. The first deliverable is a serial 6DOF kinematics core with canonical robot config, FK, numerical IK, frame/tool support, posture-aware solution selection, custom presets, and one virtual implementation preset.

The plan intentionally builds the foundation first, then solver behavior, then a project-owned virtual preset, then real vendor presets after source data is provided. SCARA, delta, parallel, and analytic IK are designed for but not implemented in phase 1 unless separately approved.

## Dependency Graph

```text
Units + Pose
    |
    v
Canonical robot model
    |
    v
Model validation + joint limits
    |
    v
Frame/tool registry
    |
    v
Forward kinematics
    |
    v
IK solver interface
    |
    v
Numerical IK solver
    |
    v
Solution ranking + posture resolver
    |
    v
Virtual preset + integration tests
    |
    v
Real preset ingestion + validation
    |
    v
Adapters: DH/URDF
```

## Architecture Decisions

- Core model is URDF-like: links, joints, joint origin transforms, axes.
- Core units are meter/radian.
- `Pose` wraps `Eigen::Isometry3d`.
- FK/IK operate only on canonical model.
- URDF and DH are adapters, not the source of truth.
- IK architecture is hybrid: numerical first, analytic plugins later.
- Phase 1 numerical IK uses adaptive damped least squares with joint-limit handling, weighted position/orientation residuals, and multi-seed search.
- `solve` returns the best solution by policy; `solveAll` returns all solutions found by the selected solver.
- Posture logic is configurable per robot family/preset.
- Test framework is Qt Test.
- MSVC is the primary Windows compiler; MinGW is a compatibility target.
- Presets are JSON files. Virtual6DofTestArm is the first required preset and has a built-in C++ fallback. Kawasaki RS007N and Nachi MZ04D are integrated after source data is provided.
- Preset JSON schema is `robot-kinematics-preset/v1` and stores solver-facing values in SI units only.
- IK/validation status uses `KinematicsStatus`.

## Phase 0: Project Foundation

### Task 0.1: Scaffold Qt/qmake Library Project

**Description:** Create the initial qmake project, source/include/test directories, and minimal build target.

**Acceptance criteria:**
- [x] `RobotKinematics.pro` exists.
- [x] Library target builds.
- [x] Test target builds.
- [x] Include/source directory structure matches the spec.

**Verification:**
- [x] Run `qmake RobotKinematics.pro`.
- [x] Run `nmake` or `mingw32-make`.
- [x] Run empty test executable.

**Dependencies:** None.

**Files likely touched:**
- `RobotKinematics.pro`
- `include/RobotKinematics/...`
- `src/...`
- `tests/...`

**Estimated scope:** Medium.

### Task 0.2: Configure Qt Test

**Description:** Wire Qt Test into the qmake project.

**Acceptance criteria:**
- [x] Qt Test is configured and documented.
- [x] One smoke test runs in CI/local command.
- [x] Test command is documented.

**Verification:**
- [x] Run test executable successfully.

**Dependencies:** Task 0.1.

**Files likely touched:**
- `RobotKinematics.pro`
- `tests/RobotKinematicsTests.pro`
- `tests/unit/...`
- `docs/robot_kinematics_spec.md`

**Estimated scope:** Small.

## Phase 1: Core Types And Canonical Model

### Task 1.1: Implement Units Helpers

**Description:** Add explicit helpers for meter/mm and radian/degree conversion.

**Acceptance criteria:**
- [x] Core helpers convert mm to m and deg to rad.
- [x] Reverse helpers convert m to mm and rad to deg.
- [x] No implicit unit conversion API is introduced.

**Verification:**
- [x] Unit tests cover representative values and edge cases.

**Dependencies:** Phase 0.

**Files likely touched:**
- `include/RobotKinematics/Core/Units.h`
- `src/Core/Units.cpp`
- `tests/unit/UnitsTests.cpp`

**Estimated scope:** Small.

### Task 1.2: Implement Pose Wrapper

**Description:** Implement `Pose` as a wrapper around `Eigen::Isometry3d` with constructors for identity, isometry, XYZRPY in m/rad, and XYZRPY in mm/deg.

**Acceptance criteria:**
- [x] `Pose::identity()` works.
- [x] `Pose::fromIsometry()` preserves transform.
- [x] XYZRPY helper uses documented rotation convention.
- [x] Quaternion accessor exists.

**Verification:**
- [x] Unit tests verify transform composition and RPY convention.

**Dependencies:** Task 1.1.

**Files likely touched:**
- `include/RobotKinematics/Core/Pose.h`
- `src/Core/Pose.cpp`
- `tests/unit/PoseTests.cpp`

**Estimated scope:** Medium.

### Task 1.3: Implement Canonical Model Types

**Description:** Define core model structs/classes for robot identity, links, joints, frames, tools, joint limits, and serial robot config.

**Acceptance criteria:**
- [x] Serial 6DOF robot config can be represented.
- [x] Joint origin transform and joint axis are modeled.
- [x] Revolute, prismatic, and fixed joint types exist.
- [x] Joint limits support min/max position and optional velocity/acceleration.

**Verification:**
- [x] Unit tests instantiate valid and invalid model examples.

**Dependencies:** Task 1.2.

**Files likely touched:**
- `include/RobotKinematics/Model/...`
- `src/Model/...`
- `tests/unit/RobotModelConfigTests.cpp`

**Estimated scope:** Medium.

### Task 1.4: Implement Model Validation

**Description:** Validate model completeness before FK/IK use.

**Acceptance criteria:**
- [x] Missing base/flange/joint data is reported.
- [x] Invalid joint axis is reported.
- [x] Invalid limits are reported.
- [x] Serial chain order can be validated.

**Verification:**
- [x] Unit tests cover valid model, missing fields, bad limits, and malformed serial chain.

**Dependencies:** Task 1.3.

**Files likely touched:**
- `include/RobotKinematics/Model/RobotModelValidator.h`
- `src/Model/RobotModelValidator.cpp`
- `tests/unit/RobotModelValidatorTests.cpp`

**Estimated scope:** Medium.

### Checkpoint: Core Model

- [x] Build succeeds.
- [x] Build is verified with MSVC.
- [x] Build compatibility is checked with MinGW before release.
- [x] Unit tests for units, pose, model, and validation pass.
- [x] No spec contract update needed for the implemented model fields.
- [X] Human review before implementing FK.

## Phase 2: FK, Joint Limits, Frames, And Tools

### Task 2.1: Implement JointVector And Joint Limit Validation

**Description:** Add joint vector representation and validation against model limits.

**Acceptance criteria:**
- [x] Joint vector validates size against robot model.
- [x] Joint position limits are checked.
- [x] Result reports which joint failed.

**Verification:**
- [x] Unit tests cover inside limit, below min, above max, and wrong dimension.

**Dependencies:** Task 1.4.

**Files likely touched:**
- `include/RobotKinematics/Core/JointVector.h`
- `include/RobotKinematics/Kinematics/JointLimitValidator.h`
- `src/Kinematics/JointLimitValidator.cpp`
- `tests/unit/JointLimitValidatorTests.cpp`

**Estimated scope:** Small.

### Task 2.2: Implement Frame And Tool Registry

**Description:** Add APIs to register, retrieve, and apply user frames and tools.

**Acceptance criteria:**
- [x] User frame can be added by id.
- [x] Tool can be added by id.
- [x] Missing id returns structured error.
- [x] Default tool can be configured.

**Verification:**
- [x] Unit tests cover add/get/default/missing behavior.

**Dependencies:** Task 1.4.

**Files likely touched:**
- `include/RobotKinematics/Model/Frame.h`
- `include/RobotKinematics/Model/Tool.h`
- `src/Model/...`
- `tests/unit/FrameToolTests.cpp`

**Estimated scope:** Small.

### Task 2.3: Implement Forward Kinematics

**Description:** Implement FK transform chaining for serial robot configs.

**Acceptance criteria:**
- [x] FK computes base-to-flange pose.
- [x] FK computes base-to-tool pose.
- [x] FK can return pose relative to requested user frame.
- [x] FK rejects invalid joint vectors.

**Verification:**
- [x] Unit tests cover simple 1-joint, 2-joint, and 6-joint fixtures.
- [x] Integration test verifies tool/frame composition.

**Dependencies:** Task 2.1, Task 2.2.

**Files likely touched:**
- `include/RobotKinematics/Kinematics/ForwardKinematics.h`
- `src/Kinematics/ForwardKinematics.cpp`
- `tests/unit/ForwardKinematicsTests.cpp`
- `tests/integration/FrameToolFkTests.cpp`

**Estimated scope:** Medium.

### Checkpoint: FK And Frames

- [x] Build succeeds.
- [x] FK tests pass.
- [x] Frame/tool behavior is documented.
- [x] Human review before IK implementation.

## Phase 3: IK Interface And Numerical Solver

### Task 3.1: Define IK Request, Result, Status, And Solver Interface

**Description:** Add public IK API types and internal `IKSolver` interface.

**Acceptance criteria:**
- [x] `IKRequest`, `IKOptions`, `IKSolution`, `IKResult`, and `IKStatus` exist.
- [x] `KinematicsStatus` includes `Ok`, `InvalidRobotConfig`, `InvalidRequest`, `FrameNotFound`, `ToolNotFound`, `JointDimensionMismatch`, `JointLimitViolation`, `TargetUnreachable`, `Singularity`, `MaxIterationsReached`, `NoConvergedSolution`, `PostureConstraintUnsatisfied`, `UnsupportedSolver`, and `NumericalError`.
- [x] `solve` and `solveAll` API shape is defined.
- [x] Failure statuses are structured.

**Verification:**
- [x] Compile-time tests or unit tests validate API creation and status handling.

**Dependencies:** Task 2.3.

**Files likely touched:**
- `include/RobotKinematics/Kinematics/InverseKinematics.h`
- `include/RobotKinematics/Solvers/IKSolver.h`
- `src/Kinematics/InverseKinematics.cpp`
- `tests/unit/IKApiTests.cpp`

**Estimated scope:** Medium.

### Task 3.2: Implement Numerical IK Solver

**Description:** Implement generic numerical IK for serial robots.

**Acceptance criteria:**
- [x] Solver minimizes position and orientation residual.
- [x] Solver respects joint limits.
- [x] Solver returns structured failure on max iterations/unreachable target.
- [x] Solver uses adaptive damped least squares.
- [x] Solver supports weighted position/orientation residuals.
- [x] Defaults are implemented: `maxIterations=200`, `maxSeeds=32`, `positionTolerance_m=1e-6`, `orientationTolerance_rad=1.7453292519943296e-5`, `initialDamping=1e-3`, `minDamping=1e-8`, `maxDamping=1e2`, `maxJointStepNorm=0.2`, `jointLimitAvoidanceWeight=0.05`.
- [x] Seed order is deterministic: previous joint, seed joint, posture-derived seeds, home, joint-limit midpoint, then deterministic low-discrepancy seeds.
- [x] Accuracy target is met for normal non-singular fixtures.

**Verification:**
- [x] Unit tests run numerical IK on simple serial fixtures.
- [x] Residual position error <= `1e-6 m` for selected non-singular cases.
- [x] Residual orientation error <= `0.001 degree` for selected non-singular cases.
- [x] Joint-vector round-trip comparisons use <= `0.0001 degree` per revolute joint by default; tests using teach-pendant/reference coordinates rounded to 2 decimal places may use <= `0.001 degree` and must document that exception.

**Dependencies:** Task 3.1.

**Files likely touched:**
- `include/RobotKinematics/Solvers/NumericalIKSolver.h`
- `src/Solvers/NumericalIKSolver.cpp`
- `tests/unit/NumericalIKSolverTests.cpp`

**Estimated scope:** Medium.

### Task 3.3: Implement Solve And SolveAll Orchestration

**Description:** Wire solver execution into `RobotKinematics` public API.

**Acceptance criteria:**
- [x] `solve(request)` returns 0 or 1 best solution.
- [x] `solveAll(request)` returns all found solutions from configured seeds.
- [x] Numerical `solveAll` is documented as found-solutions, not exhaustive.
- [x] Requests can include frame and tool ids.

**Verification:**
- [x] Integration tests cover solve, solveAll, missing tool/frame, and unreachable pose.

**Dependencies:** Task 3.2.

**Files likely touched:**
- `include/RobotKinematics/Kinematics/RobotKinematics.h`
- `include/RobotKinematics/Kinematics/SerialRobotKinematics.h`
- `src/Kinematics/RobotKinematics.cpp`
- `src/Kinematics/SerialRobotKinematics.cpp`
- `tests/integration/IKIntegrationTests.cpp`

**Estimated scope:** Medium.

### Checkpoint: Numerical IK

- [x] Build succeeds.
- [x] IK unit/integration tests pass.
- [x] Accuracy criteria are measured and documented.
- [x] Known solver limitations are documented.

## Phase 4: Posture And Solution Ranking

### Task 4.1: Implement ArmPosture And PostureResolver Interface

**Description:** Add configurable posture/branch metadata and resolver interface.

**Acceptance criteria:**
- [x] Generic `ArmPosture` type exists.
- [x] Vendor-specific posture metadata can be stored.
- [x] Resolver can map preset/vendor names to internal branch values.

**Verification:**
- [x] Unit tests cover generic and vendor-specific posture mapping.

**Dependencies:** Task 3.1.

**Files likely touched:**
- `include/RobotKinematics/Posture/ArmPosture.h`
- `include/RobotKinematics/Posture/PostureResolver.h`
- `src/Posture/...`
- `tests/unit/PostureResolverTests.cpp`

**Estimated scope:** Small.

### Task 4.2: Implement IK Solution Ranking

**Description:** Score IK candidates using seed distance, previous joint continuity, joint limit margin, and posture preference.

**Acceptance criteria:**
- [x] Score object includes cost breakdown.
- [x] `solve` chooses lowest-cost valid candidate.
- [x] `requirePosture` rejects posture mismatch.
- [x] `preferPosture` behavior is represented by cost when hard requirement is off.

**Verification:**
- [x] Unit tests cover ranking priority and deterministic tie behavior.

**Dependencies:** Task 4.1, Task 3.3.

**Files likely touched:**
- `include/RobotKinematics/Solvers/IKSolutionRanker.h`
- `src/Solvers/IKSolutionRanker.cpp`
- `tests/unit/IKSolutionRankerTests.cpp`

**Estimated scope:** Medium.

### Checkpoint: Posture-Aware IK

- [x] `solve` behavior is deterministic.
- [x] Seed/previous/posture behavior is tested.
- [x] Contract wording for "best solution" is updated if ranking changes.

## Phase 5: Presets And Custom Config

### Task 5.1: Implement Custom Preset Builder/Loader Interface

**Description:** Provide a way to construct serial robot configs programmatically and optionally from a serialized config format.

**Acceptance criteria:**
- [x] Developer can create custom serial 6DOF config.
- [x] Developer can override link transforms, joint axes, names, and limits.
- [x] Developer can load a `robot-kinematics-preset/v1` JSON preset.
- [x] JSON preset values are interpreted as SI units only.
- [x] JSON supports identity, topology, links, ordered joints, frames, tools, default tool, posture metadata, solver metadata, source references, and custom metadata.
- [x] Config validation catches malformed custom presets.

**Verification:**
- [x] Integration test builds a custom 6DOF robot and runs FK/IK.

**Dependencies:** Task 4.2.

**Files likely touched:**
- `include/RobotKinematics/Model/SerialRobotConfigBuilder.h`
- `src/Model/SerialRobotConfigBuilder.cpp`
- `tests/integration/CustomPresetTests.cpp`

**Estimated scope:** Medium.

### Task 5.2: Design And Add Virtual6DofTestArm Preset

**Description:** Add a self-designed virtual serial 6DOF preset used to validate the base code before real robot data is available.

**Acceptance criteria:**
- [x] Preset creates a valid serial 6DOF config.
- [x] Preset is available as JSON.
- [x] Preset has built-in C++ fallback.
- [x] Preset uses non-zero link offsets and non-parallel joint axes sufficient to exercise transform chaining.
- [x] Preset includes finite joint limits, home position, base frame, one user frame, default tool, and one non-default test tool.
- [x] Lefty/righty, above/below, and flip/non-flip posture classification is supported.
- [x] FK tests pass against deterministic fixture values.
- [x] IK round-trip tests pass for selected non-singular poses generated from FK, using the default joint-angle comparison tolerance of <= `0.0001 degree` per revolute joint when comparing to an expected joint vector.
- [x] JSON loader and C++ fallback produce equivalent canonical configs.

**Verification:**
- [x] Virtual preset integration tests pass.

**Dependencies:** Task 5.1.

**Files likely touched:**
- `include/RobotKinematics/Presets/Virtual6DofTestArm.h`
- `src/Presets/Virtual6DofTestArm.cpp`
- `presets/virtual_6dof_test_arm.json`
- `tests/integration/Virtual6DofTestArmTests.cpp`
- `tests/fixtures/...`

**Estimated scope:** Medium.

### Checkpoint: Base Code With Virtual Preset

- [x] Library builds.
- [x] All unit tests pass.
- [x] Virtual preset integration tests pass.
- [x] Accuracy criteria are met for normal non-singular virtual fixture poses.
- [x] Public API examples compile using Virtual6DofTestArm.
- [x] Documentation explains custom preset creation and the virtual preset purpose.

## Phase 6: Real Preset Ingestion And Validation

### Task 6.1: Add Kawasaki RS007N Preset

**Description:** Add Kawasaki RS007N after source dimensions, joint limits, and posture rules are provided.

**Status:** Blocked. Do not implement until verified dimensions, joint limits, posture rules, and source references are provided.

**Acceptance criteria:**
- [ ] Preset creates a valid serial 6DOF config.
- [ ] Preset is available as JSON.
- [ ] Preset has built-in C++ fallback.
- [ ] Source references for dimensions, joint limits, and posture definitions are recorded where available.
- [ ] Lefty/righty, above/below, and flip/non-flip posture classification is supported.
- [ ] FK tests pass against expected fixture values.
- [ ] IK round-trip tests pass for selected non-singular poses. Joint-vector comparisons use <= `0.0001 degree` per revolute joint by default, or <= `0.001 degree` only when the expected coordinate source is teach-pendant/reference data rounded to 2 decimal places and the test documents that source precision.

**Verification:**
- [ ] Preset integration tests pass.

**Dependencies:** Task 5.2 and source data from the user.

**Files likely touched:**
- `include/RobotKinematics/Presets/KawasakiRS007N.h`
- `src/Presets/KawasakiRS007N.cpp`
- `presets/kawasaki_rs007n.json`
- `tests/integration/KawasakiRS007NTests.cpp`
- `tests/fixtures/...`

**Estimated scope:** Medium.

### Task 6.2: Add Nachi MZ04D Preset

**Description:** Add Nachi MZ04D after source dimensions, joint limits, and posture rules are provided.

**Status:** Implemented from teach-pendant data (docs/preset_references/nachi-mz04d.md): kinematics, joint limits, and posture (shoulder/wrist + elbow below-side confirmed by ground-truth Arm-config set; elbow above-side inferred). FK verified for 21 flange poses and 4 tool-offset TCP poses.

**Acceptance criteria:**
- [x] Preset creates a valid serial 6DOF config.
- [x] Preset is available as JSON.
- [x] Preset has built-in C++ fallback.
- [x] Source references for dimensions and joint limits are recorded; posture source still pending.
- [x] Lefty/righty, above/below, and flip/non-flip posture classification is supported (Nachi-manual mapping: shoulder=sign(J1), wrist=sign(J5), elbow=sign(J3); below/non-flip/lefty/righty confirmed by ground-truth Arm-config set, elbow above-side inferred).
- [x] FK tests pass against expected fixture values (21 teach-pendant poses, <= 0.035 mm / 0.010 deg).
- [x] IK round-trip tests pass for selected non-singular poses. Expected joint-vector comparisons should use <= `0.0001 degree` per revolute joint unless a specific fixture depends on teach-pendant/reference coordinates rounded to 2 decimal places; those fixtures may use <= `0.001 degree` with an inline explanation.

**Verification:**
- [x] Preset integration tests pass.

**Dependencies:** Task 5.2 and source data from the user.

**Files likely touched:**
- `include/RobotKinematics/Presets/NachiMZ04D.h`
- `src/Presets/NachiMZ04D.cpp`
- `presets/nachi_mz04d.json`
- `tests/integration/NachiMZ04DTests.cpp`
- `tests/fixtures/...`

**Estimated scope:** Medium.

### Checkpoint: Real Presets

- [x] Library builds.
- [x] All unit tests pass.
- [x] All preset integration tests pass (Nachi MZ04D).
- [x] Accuracy criteria are met for normal non-singular fixtures (MZ04D FK <= 0.035 mm / 0.01 deg).
- [x] Real preset source references are recorded (Nachi MZ04D).
- [x] Documentation explains real preset limitations and source references (Nachi MZ04D).
- [x] Kawasaki RS007N remains blocked pending verified source data; Nachi MZ04D is implemented.

## Phase 7: Adapters

### Task 7.1: Implement DH/Modified DH Adapter

**Description:** Convert DH or Modified DH parameters into canonical robot config.

**Acceptance criteria:**
- [x] DH input can produce valid canonical serial config.
- [ ] Modified DH input can produce valid canonical serial config, if included.
- [x] Convention is explicitly documented.

**Verification:**
- [x] Adapter tests compare FK output with known DH examples.

**Dependencies:** Phase 5.

**Files likely touched:**
- `include/RobotKinematics/Adapters/DhAdapter.h`
- `src/Adapters/DhAdapter.cpp`
- `tests/unit/DhAdapterTests.cpp`

**Estimated scope:** Medium.

### Task 7.2: Implement URDF Export Best-Effort

**Description:** Export canonical model to URDF-like representation where possible.

**Acceptance criteria:**
- [x] Links and joints export.
- [x] Joint origins, axes, and limits export.
- [x] Non-URDF metadata is not silently lost without warning.

**Verification:**
- [x] Export tests inspect generated URDF structure.

**Dependencies:** Phase 5.

**Files likely touched:**
- `include/RobotKinematics/Adapters/UrdfAdapter.h`
- `src/Adapters/UrdfAdapter.cpp`
- `tests/unit/UrdfAdapterTests.cpp`

**Estimated scope:** Medium.

### Task 7.3: Implement URDF Import

**Description:** Import URDF link/joint tree into canonical model.

**Acceptance criteria:**
- [x] Serial chain URDF can import into canonical config.
- [x] Unsupported structures return clear errors.
- [x] Metadata not present in URDF can be supplied separately later.

**Verification:**
- [x] Import tests load URDF fixtures and run FK.

**Dependencies:** Task 7.2 or independent if parser selected earlier.

**Files likely touched:**
- `include/RobotKinematics/Adapters/UrdfAdapter.h`
- `src/Adapters/UrdfAdapter.cpp`
- `tests/unit/UrdfAdapterTests.cpp`
- `tests/fixtures/urdf/...`

**Estimated scope:** Medium.

## Phase 8: Analytic IK Plugin

### Task 8.1: Define Analytic Solver Capability Detection

**Description:** Let solvers declare whether they support a given robot morphology.

**Acceptance criteria:**
- [x] Solver can return supported/unsupported for model config.
- [x] Unsupported analytic solver falls back to numerical solver if configured.

**Verification:**
- [x] Unit tests cover supported and unsupported capability checks.

**Dependencies:** Phase 5.

**Files likely touched:**
- `include/RobotKinematics/Solvers/AnalyticIKSolver.h`
- `src/Solvers/AnalyticIKSolver.cpp`
- `tests/unit/AnalyticIKSolverTests.cpp`

**Estimated scope:** Small.

### Task 8.2: Implement Analytic IK For Supported 6DOF Morphology

**Description:** Add analytic IK for supported 6DOF serial arms, likely spherical wrist morphology.

**Acceptance criteria:**
- [x] Solver returns discrete branch solutions for supported models (up to 8 for spherical-wrist articulated arms).
- [x] Posture/branch metadata maps to analytic solutions (shoulder/elbow/wrist sign per branch).
- [x] `solveAll` can declare stronger solution coverage for supported models (exhaustive closed-form branches).

**Verification:**
- [x] Analytic IK tests compare all returned branches against FK residual.

**Note:** Supported morphology = articulated arm (J1 ⊥ J2, J2 ∥ J3, no shoulder offset) with a
spherical wrist whose reduction is a proper Z–Y–Z Euler problem (validated against Nachi MZ04D
to machine precision). Other morphologies report unsupported and fall back to the numerical solver.

**Dependencies:** Task 8.1.

**Files likely touched:**
- `include/RobotKinematics/Solvers/Analytic6DofSphericalWristSolver.h`
- `src/Solvers/Analytic6DofSphericalWristSolver.cpp`
- `tests/unit/Analytic6DofSphericalWristSolverTests.cpp`

**Estimated scope:** Medium to Large. Break further before implementation.

## Risks And Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Preset dimensions or joint limits are wrong | High | Validate base behavior with Virtual6DofTestArm first; require source reference for each real preset value and fixture tests. |
| `0.001 mm` IK tolerance is too strict for numerical solver | High | Apply only to normal non-singular test cases; track residuals; tune solver/options; document exclusions. |
| `0.001 degree` orientation tolerance is too strict for some numerical cases | Medium | Apply to normal non-singular test cases; tune adaptive damping and residual weights; document exclusions. |
| `0.0001 degree` joint round-trip tolerance is too strict for rounded teach-pendant fixtures | Medium | Use <= `0.0001 degree` for synthetic/generated fixtures; allow <= `0.001 degree` only when source coordinates are rounded to 2 decimal places and the test documents that precision limit. |
| `solveAll` expectation becomes "mathematically all" | High | Document found-solutions semantics for numerical solver and exhaustive-only claim for analytic solvers. |
| Unit confusion between mm/degree and m/radian | High | No implicit conversion; helper names include unit; tests cover conversions. |
| RPY convention mismatch | Medium | Document exact convention; add tests with known rotations. |
| Posture semantics differ by vendor | Medium | Keep base posture generic; implement per-preset resolver. |
| URDF cannot represent all canonical metadata | Medium | Treat URDF as adapter only; warn on metadata loss. |
| Scope expands to many robot families too early | High | Keep phase 1 serial 6DOF only; require approval for new robot family implementation. |

## Parallelization Opportunities

Safe to parallelize after contracts are defined:

- Units/Pose tests and model type implementation.
- Virtual preset fixture design after schema is stable.
- Preset data collection for Kawasaki and Nachi after base code starts.
- Documentation examples.
- DH adapter after canonical model is stable.

Must be sequential:

- Pose and units before model/FK.
- Canonical model before FK.
- FK before IK.
- IK API before numerical solver.
- Ranking/posture before final `solve` behavior.
- Virtual preset before real preset validation.

Needs coordination:

- Preset data and posture resolver.
- Solver residual tolerance and acceptance tests.
- URDF adapter and canonical model field changes.

## Open Questions To Resolve Later

- Exact source documents/config files for Kawasaki RS007N dimensions, joint limits, and posture definitions.

## Validation Follow-Ups

- Check whether solver defaults need tuning after empirical validation.
- How much analytic IK work should be pulled earlier if numerical multi-seed posture coverage is not enough for Kawasaki/Nachi.
