#include <QtTest/QtTest>

#include <iostream>

int runSmokeTests(int argc, char** argv);
int runDhAdapterTests(int argc, char** argv);
int runUnitsTests(int argc, char** argv);
int runPoseTests(int argc, char** argv);
int runRobotModelConfigTests(int argc, char** argv);
int runRobotModelValidatorTests(int argc, char** argv);
int runJointLimitValidatorTests(int argc, char** argv);
int runFrameToolTests(int argc, char** argv);
int runForwardKinematicsTests(int argc, char** argv);
int runRobot3DVisualizerLogicTests(int argc, char** argv);
int runFrameToolFkTests(int argc, char** argv);
int runIKApiTests(int argc, char** argv);
int runIKSolutionRankerTests(int argc, char** argv);
int runNumericalIKSolverTests(int argc, char** argv);
int runPostureResolverTests(int argc, char** argv);
int runCustomPresetTests(int argc, char** argv);
int runIKIntegrationTests(int argc, char** argv);
int runVirtual6DofTestArmTests(int argc, char** argv);
int runNachiMZ04DTests(int argc, char** argv);
int runAnalyticIKSolverTests(int argc, char** argv);
int runUrdfAdapterTests(int argc, char** argv);

namespace {
int runSuite(const char* name, int (*suite)(int, char**), int argc, char** argv)
{
    std::cout << "Running " << name << "..." << std::endl;
    const int failures = suite(argc, argv);
    std::cout << name << ": " << (failures == 0 ? "PASS" : "FAIL");
    if (failures != 0) {
        std::cout << " (" << failures << " failing test function(s))";
    }
    std::cout << std::endl;
    return failures;
}
}

int main(int argc, char** argv)
{
    int status = 0;
    status |= runSuite("SmokeTests", runSmokeTests, argc, argv);
    status |= runSuite("DhAdapterTests", runDhAdapterTests, argc, argv);
    status |= runSuite("UnitsTests", runUnitsTests, argc, argv);
    status |= runSuite("PoseTests", runPoseTests, argc, argv);
    status |= runSuite("RobotModelConfigTests", runRobotModelConfigTests, argc, argv);
    status |= runSuite("RobotModelValidatorTests", runRobotModelValidatorTests, argc, argv);
    status |= runSuite("JointLimitValidatorTests", runJointLimitValidatorTests, argc, argv);
    status |= runSuite("FrameToolTests", runFrameToolTests, argc, argv);
    status |= runSuite("ForwardKinematicsTests", runForwardKinematicsTests, argc, argv);
    status |= runSuite("Robot3DVisualizerLogicTests", runRobot3DVisualizerLogicTests, argc, argv);
    status |= runSuite("FrameToolFkTests", runFrameToolFkTests, argc, argv);
    status |= runSuite("IKApiTests", runIKApiTests, argc, argv);
    status |= runSuite("IKSolutionRankerTests", runIKSolutionRankerTests, argc, argv);
    status |= runSuite("NumericalIKSolverTests", runNumericalIKSolverTests, argc, argv);
    status |= runSuite("PostureResolverTests", runPostureResolverTests, argc, argv);
    status |= runSuite("CustomPresetTests", runCustomPresetTests, argc, argv);
    status |= runSuite("IKIntegrationTests", runIKIntegrationTests, argc, argv);
    status |= runSuite("Virtual6DofTestArmTests", runVirtual6DofTestArmTests, argc, argv);
    status |= runSuite("NachiMZ04DTests", runNachiMZ04DTests, argc, argv);
    status |= runSuite("AnalyticIKSolverTests", runAnalyticIKSolverTests, argc, argv);
    status |= runSuite("UrdfAdapterTests", runUrdfAdapterTests, argc, argv);
    return status;
}
