#include <RobotKinematics/Solvers/Analytic6DofSphericalWristSolver.h>

#include <RobotKinematics/Kinematics/ForwardKinematics.h>

namespace RobotKinematics {

namespace {
JointVector homeJoint(const SerialRobotConfig& config)
{
    Eigen::VectorXd q(ForwardKinematics::movableJointCount(config));
    int index = 0;
    for (const Joint& joint : config.joints) {
        if (joint.type == JointType::Revolute || joint.type == JointType::Prismatic) {
            q[index++] = joint.home;
        }
    }
    return JointVector(q);
}
}

const char* Analytic6DofSphericalWristSolver::name() const
{
    return "analytic_6dof_spherical_wrist";
}

bool Analytic6DofSphericalWristSolver::supportsModel(const SerialRobotConfig& config) const
{
    if (config.topology != RobotTopologyType::Serial || ForwardKinematics::movableJointCount(config) != 6) {
        return false;
    }
    for (const Joint& joint : config.joints) {
        if (joint.type == JointType::Prismatic) {
            return false;
        }
    }

    const FkChain chain = ForwardKinematics::computeChain(config, homeJoint(config));
    if (chain.joints.size() != 6) {
        return false;
    }
    const Eigen::Vector3d wrist = chain.joints[3].originInBase;
    return (chain.joints[4].originInBase - wrist).norm() <= 1e-9
           && (chain.joints[5].originInBase - wrist).norm() <= 1e-9;
}

IKResult Analytic6DofSphericalWristSolver::solve(const SerialRobotConfig& config,
                                                 const IKSolveContext& context) const
{
    IKResult result = solveAll(config, context);
    if (result.solutions.size() > 1) {
        result.solutions.resize(1);
    }
    return result;
}

IKResult Analytic6DofSphericalWristSolver::solveAll(const SerialRobotConfig& config,
                                                    const IKSolveContext&) const
{
    if (!supportsModel(config)) {
        return IKResult{IKStatus::UnsupportedSolver, {}, "analytic spherical-wrist solver does not support this model"};
    }
    return IKResult{IKStatus::UnsupportedSolver, {}, "analytic spherical-wrist closed-form solve is not implemented yet"};
}

} // namespace RobotKinematics
