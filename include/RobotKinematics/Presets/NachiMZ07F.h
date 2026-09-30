#pragma once

#include <RobotKinematics/Model/RobotModelConfig.h>

namespace RobotKinematics::Presets {

// Nachi MZ07F serial 6DOF preset.
//
// Link dimensions were verified against teach-pendant measurements (joint angles vs.
// tool-flange poses) recorded in docs/preset_references/nachi-mz07f.md. The canonical
// joint origins below are the standard-DH table
//
//        theta   d(mm)   alpha   a(mm)
//   J1     0     355.0   +90      50      (a = 50 mm shoulder offset between J1 and J2 axes)
//   J2     0       0       0     330
//   J3     0       0     +90      45
//   J4     0     340     -90       0
//   J5     0       0     +90       0
//   J6     0      78       0       0
//
// expressed as canonical link transforms (all joints revolute about local Z). This
// table reproduces all 6 reference poses to <= 0.07 mm and <= 0.013 deg.
//
// Because of the J1/J2 shoulder offset, the analytic spherical-wrist IK plugin does not
// support this model; IK routes to the numerical solver.
//
// Joint position limits are the actual limits provided for the reference robot
// (docs/preset_references/nachi-mz07f.md). Posture labels use the same Nachi rules as
// NachiMZ04D: shoulder = sign(J1) (J1<0 righty, J1>0 lefty), wrist = sign(J5)
// (J5<0 flip, J5>0 non-flip), elbow = sign(J3) (J3<0 above, J3>0 below).
SerialRobotConfig nachiMZ07F();

} // namespace RobotKinematics::Presets
