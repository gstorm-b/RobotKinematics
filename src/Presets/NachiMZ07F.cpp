#include <RobotKinematics/Presets/NachiMZ07F.h>

#include <RobotKinematics/Core/Pose.h>

#include <cmath>
#include <optional>

namespace RobotKinematics::Presets {

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi / 2.0;

constexpr double deg(double d) { return d * kPi / 180.0; }

Joint revolute(const std::string& id,
               const std::string& parent,
               const std::string& child,
               const Pose& origin,
               const JointLimits& limits)
{
    Joint joint;
    joint.id = id;
    joint.type = JointType::Revolute;
    joint.parentLinkId = parent;
    joint.childLinkId = child;
    joint.origin = origin;
    joint.axis = Eigen::Vector3d::UnitZ();
    joint.limits = limits;
    joint.home = 0.0;
    joint.aliases = {"JT" + id.substr(1)};
    return joint;
}
} // namespace

SerialRobotConfig nachiMZ07F()
{
    SerialRobotConfig config;
    config.identity = RobotIdentity{"Nachi", "MZ07F", "Nachi MZ07F", "1.0.0"};
    config.topology = RobotTopologyType::Serial;
    config.dof = 6;
    config.links = {{"base_link"}, {"link_1"}, {"link_2"}, {"link_3"}, {"link_4"}, {"link_5"}, {"flange"}};
    config.frames.baseLinkId = "base_link";
    config.frames.flangeLinkId = "flange";
    config.frames.userFrames.push_back(UserFrame{
        "robot_frame",
        "base_link",
        Pose::fromXYZRPY_m_rad(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    });
    config.frames.userFrames.push_back(UserFrame{
        "ceiling_frame",
        "base_link",
        Pose::fromXYZRPY_mm_deg(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
    });

    // Canonical link transforms derived from the standard-DH table (theta=0 for every joint;
    // lengths in meters). See NachiMZ07F.h for the DH table.
    // Joint position limits are from docs/preset_references/nachi-mz07f.md.
    config.joints = {
        revolute("J1", "base_link", "link_1", Pose::fromXYZRPY_m_rad(0.0, 0.0, 0.355, 0.0, 0.0, 0.0),
                 JointLimits{deg(-170.0), deg(170.0), std::nullopt, std::nullopt}),
        revolute("J2", "link_1", "link_2", Pose::fromXYZRPY_m_rad(0.050, 0.0, 0.0, kHalfPi, 0.0, 0.0),
                 JointLimits{deg(-45.0), deg(170.0), std::nullopt, std::nullopt}),
        revolute("J3", "link_2", "link_3", Pose::fromXYZRPY_m_rad(0.330, 0.0, 0.0, 0.0, 0.0, 0.0),
                 JointLimits{deg(-65.0), deg(190.0), std::nullopt, std::nullopt}),
        revolute("J4", "link_3", "link_4", Pose::fromXYZRPY_m_rad(0.045, -0.340, 0.0, kHalfPi, 0.0, 0.0),
                 JointLimits{deg(-190.0), deg(190.0), std::nullopt, std::nullopt}),
        revolute("J5", "link_4", "link_5", Pose::fromXYZRPY_m_rad(0.0, 0.0, 0.0, -kHalfPi, 0.0, 0.0),
                 JointLimits{deg(-120.0), deg(120.0), std::nullopt, std::nullopt}),
        revolute("J6", "link_5", "flange", Pose::fromXYZRPY_m_rad(0.0, -0.078, 0.0, kHalfPi, 0.0, 0.0),
                 JointLimits{deg(-360.0), deg(360.0), std::nullopt, std::nullopt}),
    };

    config.tools = {
        Tool{"default", "Default Tool", Pose::identity()},
    };
    config.defaultToolId = "default";

    // Posture mapping follows the same Nachi rules as NachiMZ04D (see NachiMZ04D.cpp):
    //   shoulder: J1 < 0 -> righty (-1), J1 > 0 -> lefty (+1)
    //   wrist:    J5 < 0 -> flip   (-1), J5 > 0 -> non-flip (+1)
    //   elbow:    J3 < 0 -> above  (-1), J3 > 0 -> below (+1)
    // Assumed identical to MZ04D (same MZ series and controller); not separately measured.
    config.posture.resolver = "serial_6dof_shoulder_elbow_wrist";
    config.posture.labels["shoulder"] = PostureLabelAxis{"righty", "lefty"};
    config.posture.labels["elbow"] = PostureLabelAxis{"above", "below"};
    config.posture.labels["wrist"] = PostureLabelAxis{"flip", "non-flip"};

    config.solver.defaultSolver = "adaptive_damped_least_squares";
    config.solver.numericParameters["maxIterations"] = 200.0;
    config.solver.numericParameters["maxSeeds"] = 32.0;
    config.solver.numericParameters["positionTolerance_m"] = 0.000001;
    config.solver.numericParameters["orientationTolerance_rad"] = 0.000017453292519943296;

    config.sources.push_back(SourceReference{
        "field_measurement",
        "Nachi MZ07F teach-pendant joint/flange pose measurements and actual joint limits",
        "docs/preset_references/nachi-mz07f.md",
        {"dimensions", "joint_limits"},
    });
    config.metadata["collisionProfile"] = "presets/Nachi/MZ07F/nachi_mz07f_collision.json";
    config.metadata["meshCollisionProfile"] = "presets/Nachi/MZ07F/nachi_mz07f_mesh_collision.json";
    config.metadata["kinematics_source"] = "dimensions_verified_against_teach_pendant_poses";
    config.metadata["joint_limits_status"] = "actual_limits_from_reference_robot";
    config.metadata["posture_status"] = "assumed_same_as_mz04d";
    return config;
}

} // namespace RobotKinematics::Presets
