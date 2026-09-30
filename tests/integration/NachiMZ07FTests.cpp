#include "NachiMZ07FTests.h"

#include <RobotKinematics/Collision/CollisionBackend.h>
#include <RobotKinematics/Collision/CollisionChecker.h>
#include <RobotKinematics/Collision/CollisionProfileJsonLoader.h>
#include <RobotKinematics/Collision/CollisionProfileValidator.h>
#include <RobotKinematics/Collision/MeshCollisionProfileJsonLoader.h>
#include <RobotKinematics/Collision/MeshCollisionProfileValidator.h>
#include <RobotKinematics/Collision/StlMeshLoader.h>
#include <RobotKinematics/Kinematics/ForwardKinematics.h>
#include <RobotKinematics/Kinematics/SerialRobotKinematics.h>
#include <RobotKinematics/Model/RobotModelValidator.h>
#include <RobotKinematics/Posture/PostureResolver.h>
#include <RobotKinematics/Presets/NachiMZ07F.h>
#include <RobotKinematics/Presets/PresetJsonLoader.h>
#include <RobotKinematics/Solvers/Analytic6DofSphericalWristSolver.h>

#include <QFile>
#include <QString>
#include <QtTest/QtTest>

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace RobotKinematics;

namespace {
constexpr double kRadToDeg = 180.0 / 3.141592653589793238462643383279502884;

// One teach-pendant reference measurement (tool-flange pose, robot base frame).
// Joints are in degrees. The pose orientation is reported by the Nachi pendant in
// Z-first order: (yaw about Z, pitch about Y, roll about X), all degrees.
struct TeachPoint {
    std::array<double, 6> jointsDeg;
    double x_mm, y_mm, z_mm;
    double yawZ_deg, pitchY_deg, rollX_deg;
};

// docs/preset_references/nachi-mz07f.md
const std::array<TeachPoint, 6> kTeachPoints = {{
    {{21.4862, 64.4801, -41.7744, -180.007, 22.7216, -158.513}, 339.525, 133.617, 278.473, 0.000535733, 0.0236197, 179.986},
    {{18.6965, 68.6763, -39.6996, -158.418, 38.2621, -180.401}, 339.486, 133.658, 311.388, 1.99842, 10.7741, -169.478},
    {{4.98917, 73.5581, -44.4732, -189.95, 35.3251, -153.57}, 339.5, 21.8373, 319.019, -13.2406, 3.7502, 172.732},
    {{-20.077, 72.1081, -41.8598, -0.0550513, -30.2747, -20.0276}, 339.503, -124.14, 319.977, -0.00232496, 0.038313, 179.985},
    {{-17.8857, 69.2501, -38.4703, -14.4347, -45.5303, -25.5808}, 339.474, -124.15, 319.966, 19.152, 17.1551, 179.99},
    {{-25.4971, 70.7751, -42.0408, 44.0796, -38.9396, -79.9948}, 339.49, -124.109, 319.939, 14.2266, -16.2692, -159.49},
}};

std::string presetFilePath(const std::string& fileName)
{
    const std::string candidates[] = {
        "../presets/Nachi/MZ07F/" + fileName,
        "presets/Nachi/MZ07F/" + fileName,
        "../../presets/Nachi/MZ07F/" + fileName,
    };
    for (const std::string& candidate : candidates) {
        if (QFile(QString::fromStdString(candidate)).exists()) {
            return candidate;
        }
    }
    return candidates[0];
}

std::string nachiPresetPath()
{
    return presetFilePath("nachi_mz07f.json");
}

bool poseNear(const Pose& a, const Pose& b, double tolerance = 1e-12)
{
    return (a.isometry().matrix() - b.isometry().matrix()).norm() <= tolerance;
}

JointVector jointsFromDegrees(const std::array<double, 6>& d)
{
    return JointVector::fromDegrees({d[0], d[1], d[2], d[3], d[4], d[5]});
}
} // namespace

void NachiMZ07FTests::fallbackPresetIsValidAndHasRequiredMetadata()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const ModelValidationResult validation = RobotModelValidator::validateSerialRobotConfig(config);

    QVERIFY2(validation.ok(), validation.issues.empty() ? "" : validation.issues.front().message.c_str());
    QCOMPARE(config.identity.vendor, std::string("Nachi"));
    QCOMPARE(config.identity.model, std::string("MZ07F"));
    QCOMPARE(config.dof, 6);
    QCOMPARE(config.joints.size(), std::size_t(6));
    QCOMPARE(config.tools.size(), std::size_t(1));
    QVERIFY(!config.frames.userFrames.empty());
    QCOMPARE(config.posture.resolver, std::string("serial_6dof_shoulder_elbow_wrist"));
    QVERIFY(!config.sources.empty());
}

void NachiMZ07FTests::jsonPresetMatchesCppFallbackForSolverFacingFields()
{
    const Result<SerialRobotConfig> loaded = PresetJsonLoader::loadFile(nachiPresetPath());
    QVERIFY2(loaded.ok(), loaded.message.c_str());

    const SerialRobotConfig fallback = Presets::nachiMZ07F();
    const SerialRobotConfig& json = loaded.value;

    QCOMPARE(json.identity.model, fallback.identity.model);
    QCOMPARE(json.dof, fallback.dof);
    QCOMPARE(json.joints.size(), fallback.joints.size());
    QCOMPARE(json.frames.baseLinkId, fallback.frames.baseLinkId);
    QCOMPARE(json.frames.flangeLinkId, fallback.frames.flangeLinkId);
    QCOMPARE(json.defaultToolId, fallback.defaultToolId);
    QCOMPARE(json.posture.resolver, fallback.posture.resolver);

    for (std::size_t i = 0; i < fallback.joints.size(); ++i) {
        QCOMPARE(json.joints[i].id, fallback.joints[i].id);
        QCOMPARE(json.joints[i].parentLinkId, fallback.joints[i].parentLinkId);
        QCOMPARE(json.joints[i].childLinkId, fallback.joints[i].childLinkId);
        QVERIFY((json.joints[i].axis - fallback.joints[i].axis).norm() <= 1e-12);
        QVERIFY(poseNear(json.joints[i].origin, fallback.joints[i].origin));
        QVERIFY(json.joints[i].limits.has_value());
        QVERIFY(fallback.joints[i].limits.has_value());
        QVERIFY(std::abs(json.joints[i].limits->lower - fallback.joints[i].limits->lower) <= 1e-9);
        QVERIFY(std::abs(json.joints[i].limits->upper - fallback.joints[i].limits->upper) <= 1e-9);
    }

    QCOMPARE(json.frames.userFrames.size(), fallback.frames.userFrames.size());
    for (std::size_t i = 0; i < fallback.frames.userFrames.size(); ++i) {
        QCOMPARE(json.frames.userFrames[i].id, fallback.frames.userFrames[i].id);
        QVERIFY(poseNear(json.frames.userFrames[i].transform, fallback.frames.userFrames[i].transform));
    }
}

void NachiMZ07FTests::forwardKinematicsMatchesTeachPendantPoses()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();

    // The nominal DH table reproduces these measurements to <= 0.065 mm / <= 0.0124 deg.
    // The pendant reports positions to ~0.001 mm but only 6 points near one region were
    // recorded, so the lengths are kept at their nominal values rather than fitted; keep the
    // tolerances close to that documented residual so regressions do not hide.
    const double positionTol_m = 7.0e-5;                 // 0.07 mm
    const double orientationTol_rad = 0.013 / kRadToDeg; // 0.013 deg

    for (std::size_t i = 0; i < kTeachPoints.size(); ++i) {
        const TeachPoint& p = kTeachPoints[i];
        const Pose actual = ForwardKinematics::flangePose(config, jointsFromDegrees(p.jointsDeg));
        // Pose::fromXYZRPY_mm_deg takes (roll about X, pitch about Y, yaw about Z); map the
        // pendant's Z-first tuple accordingly.
        const Pose expected = Pose::fromXYZRPY_mm_deg(p.x_mm, p.y_mm, p.z_mm, p.rollX_deg, p.pitchY_deg, p.yawZ_deg);

        const double posErr = (actual.translation_m() - expected.translation_m()).norm();
        const double oriErr = actual.rotationQuaternion().angularDistance(expected.rotationQuaternion());

        QVERIFY2(posErr <= positionTol_m,
                 qPrintable(QString("point %1 position error %2 mm").arg(int(i + 1)).arg(posErr * 1000.0)));
        QVERIFY2(oriErr <= orientationTol_rad,
                 qPrintable(QString("point %1 orientation error %2 deg").arg(int(i + 1)).arg(oriErr * kRadToDeg)));
    }
}

void NachiMZ07FTests::postureClassificationUsesNachiLabels()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const std::unique_ptr<PostureResolver> resolver = PostureResolverFactory::create(config);
    QVERIFY(resolver != nullptr);

    const auto nameFor = [&](const std::string& axis, const std::optional<int>& branch) -> std::string {
        const auto it = config.posture.labels.find(axis);
        if (it == config.posture.labels.end() || !branch.has_value()) {
            return std::string();
        }
        return *branch == -1 ? it->second.negative : it->second.positive;
    };

    // Posture rules are assumed identical to MZ04D (not separately measured on MZ07F), so
    // these cases check the configured sign-to-label mapping on the reference joint sets.
    struct Case {
        std::array<double, 6> jointsDeg;
        const char* shoulder;
        const char* elbow;
        const char* wrist;
    };
    const std::array<Case, 3> cases = {{
        {kTeachPoints[0].jointsDeg, "lefty", "above", "non-flip"},
        {kTeachPoints[3].jointsDeg, "righty", "above", "flip"},
        {{10.0, 60.0, 30.0, 0.0, 40.0, 0.0}, "lefty", "below", "non-flip"},
    }};

    for (const Case& c : cases) {
        const Result<ArmPosture> posture = resolver->classify(config, jointsFromDegrees(c.jointsDeg));
        QVERIFY(posture.ok());
        QCOMPARE(nameFor("shoulder", posture.value.shoulder), std::string(c.shoulder));
        QCOMPARE(nameFor("elbow", posture.value.elbow), std::string(c.elbow));
        QCOMPARE(nameFor("wrist", posture.value.wrist), std::string(c.wrist));
    }
}

void NachiMZ07FTests::presetRunsFkAndSeededIkRoundTrip()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const SerialRobotKinematics robot(config);

    // Use a non-singular reference measurement (point 1) as the target and a perturbed seed.
    const JointVector reference = jointsFromDegrees(kTeachPoints[0].jointsDeg);
    const Pose target = ForwardKinematics::flangePose(config, reference);

    IKRequest request;
    request.targetPose = target;
    request.seedJoint = jointsFromDegrees({23.0, 62.0, -40.0, -178.0, 25.0, -160.0});

    const IKResult result = robot.solve(request);
    QVERIFY(result.ok());

    const Pose solved = ForwardKinematics::flangePose(config, result.best().joints);
    QVERIFY((solved.translation_m() - target.translation_m()).norm() <= 1e-6);
    QVERIFY(solved.rotationQuaternion().angularDistance(target.rotationQuaternion()) <= 1.7453292519943296e-5);
}

void NachiMZ07FTests::analyticSolverRejectsShoulderOffsetModel()
{
    // The 50 mm J1/J2 shoulder offset means the J1 and J2 axes do not intersect, which is outside
    // the analytic spherical-wrist plugin's supported morphology; IK must use the numerical solver.
    const Analytic6DofSphericalWristSolver solver;
    QVERIFY(!solver.supportsModel(Presets::nachiMZ07F()));
}

void NachiMZ07FTests::primitiveCollisionProfileIsValidAndClearAtReferencePoses()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const Result<CollisionProfile> loaded =
        CollisionProfileJsonLoader::loadFile(presetFilePath("nachi_mz07f_collision.json"));
    QVERIFY2(loaded.ok(), loaded.message.c_str());
    QCOMPARE(loaded.value.robotModel, config.identity.model);

    const CollisionProfileValidationResult validation = CollisionProfileValidator::validate(config, loaded.value);
    QVERIFY2(validation.ok(), validation.issues.empty() ? "" : validation.issues.front().message.c_str());

    // The conservative primitives must not report self-collision at home, at the example
    // midpoint preset, or at any recorded teach-pendant pose.
    std::vector<std::array<double, 6>> poses = {
        {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 90.0, 0.0, 0.0, 0.0, 0.0}},
    };
    for (const TeachPoint& p : kTeachPoints) {
        poses.push_back(p.jointsDeg);
    }

    for (std::size_t i = 0; i < poses.size(); ++i) {
        CollisionCheckRequest request;
        request.joints = jointsFromDegrees(poses[i]);
        const CollisionCheckResult result = CollisionChecker::check(config, loaded.value, request);
        QCOMPARE(result.status, KinematicsStatus::Ok);

        QString colliding;
        for (const CollisionPairResult& pair : result.pairs) {
            if (pair.colliding) {
                colliding += QString::fromStdString(pair.geometryA + "/" + pair.geometryB + " ");
            }
        }
        QVERIFY2(!result.hasCollision,
                 qPrintable(QString("reference pose %1 collides: %2").arg(int(i)).arg(colliding)));
    }
}

void NachiMZ07FTests::meshCollisionProfileLoadsMeterStlAssets()
{
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const Result<MeshCollisionProfile> loaded =
        MeshCollisionProfileJsonLoader::loadFile(presetFilePath("nachi_mz07f_mesh_collision.json"));
    QVERIFY2(loaded.ok(), loaded.message.c_str());
    QCOMPARE(loaded.value.robotModel, config.identity.model);
    QCOMPARE(loaded.value.meshes.size(), std::size_t(7));

    const MeshCollisionProfileValidationResult validation =
        MeshCollisionProfileValidator::validate(config, loaded.value);
    QVERIFY2(validation.ok(), validation.issues.empty() ? "" : validation.issues.front().message.c_str());

    // The MZ07F STL assets are authored in meters (unlike the millimeter MZ04 assets), so every
    // mesh must declare sourceUnits "m" with a unit scale and produce a sub-meter bounding box.
    for (const MeshCollisionGeometry& mesh : loaded.value.meshes) {
        QCOMPARE(mesh.sourceUnits, MeshSourceUnits::Meters);
        QCOMPARE(mesh.scaleToMeters, 1.0);

        const Result<TriangleMesh> stl = StlMeshLoader::loadFile(mesh.path, StlMeshLoadOptions{mesh.scaleToMeters, false});
        QVERIFY2(stl.ok(), qPrintable(QString("%1: %2").arg(QString::fromStdString(mesh.path),
                                                             QString::fromStdString(stl.message))));
        QVERIFY(stl.value.statistics.triangleCount > 0);

        double largestExtent_m = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            largestExtent_m = std::max(largestExtent_m,
                                       stl.value.statistics.maximumBounds_m[axis] -
                                           stl.value.statistics.minimumBounds_m[axis]);
        }
        QVERIFY2(largestExtent_m > 0.01 && largestExtent_m < 0.6,
                 qPrintable(QString("%1 extent %2 m").arg(QString::fromStdString(mesh.id)).arg(largestExtent_m)));
    }
}

void NachiMZ07FTests::meshToLinkReproducesStepAssemblyPlacement()
{
    // presets/Nachi/MZ07F/MZ07F.step is an assembly (millimeters, Y-up) of the seven parts the
    // STL assets were exported from, modeled in the Nachi reference posture: upper arm vertical,
    // forearm horizontal, i.e. model joints (0, 90, 0, 0, 0, 0) deg. Every component is placed
    // with rotation diag(-1, 1, -1); mapping the Y-up assembly into the Z-up base frame (arm
    // along +X at J1 = 0) turns that into Rx(+90 deg) for every part, and turns the component
    // translations into the base-frame origins below. FK(link) * meshToLink must reproduce them.
    struct ExpectedPart {
        const char* meshId;
        double x_mm, y_mm, z_mm;
    };
    const std::array<ExpectedPart, 7> expectedParts = {{
        {"base_mesh", 0.0, 0.0, 0.0},
        {"j1_mesh", 0.0, 0.0, 192.0},
        {"j2_mesh", 50.0, 0.0, 355.0},
        {"j3_mesh", 50.0, 0.0, 685.0},
        {"j4_mesh", 141.0, 0.0, 730.0},
        {"j5_mesh", 390.0, 0.0, 730.0},
        {"j6_mesh", 458.5, 0.0, 730.0},
    }};

    const Result<MeshCollisionProfile> loaded =
        MeshCollisionProfileJsonLoader::loadFile(presetFilePath("nachi_mz07f_mesh_collision.json"));
    QVERIFY2(loaded.ok(), loaded.message.c_str());

    const SerialRobotConfig config = Presets::nachiMZ07F();
    const FkChain chain = ForwardKinematics::computeChain(config, jointsFromDegrees({0.0, 90.0, 0.0, 0.0, 0.0, 0.0}));

    for (const ExpectedPart& part : expectedParts) {
        const auto meshIt = std::find_if(loaded.value.meshes.begin(), loaded.value.meshes.end(),
                                         [&](const MeshCollisionGeometry& mesh) { return mesh.id == part.meshId; });
        QVERIFY2(meshIt != loaded.value.meshes.end(), part.meshId);
        const auto linkIt = chain.linkPosesInBase.find(meshIt->linkId);
        QVERIFY2(linkIt != chain.linkPosesInBase.end(), part.meshId);

        const Pose actual = linkIt->second * meshIt->meshToLink;
        const Pose expected = Pose::fromXYZRPY_mm_deg(part.x_mm, part.y_mm, part.z_mm, 90.0, 0.0, 0.0);
        QVERIFY2(poseNear(actual, expected, 1e-9), part.meshId);
    }
}

void NachiMZ07FTests::meshBackendReportsNoSelfCollisionAtReferencePosesWhenCompiled()
{
#ifndef ROBOTKINEMATICS_HAVE_COAL_MESH_BACKEND
    QSKIP("Coal mesh backend is not compiled in this build");
#else
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const Result<MeshCollisionProfile> loaded =
        MeshCollisionProfileJsonLoader::loadFile(presetFilePath("nachi_mz07f_mesh_collision.json"));
    QVERIFY2(loaded.ok(), loaded.message.c_str());

    // The CAD reference posture and every recorded teach-pendant pose are physically reachable
    // on the real robot, so the original STL meshes must not overlap there.
    std::vector<std::array<double, 6>> poses = {{{0.0, 90.0, 0.0, 0.0, 0.0, 0.0}}};
    for (const TeachPoint& p : kTeachPoints) {
        poses.push_back(p.jointsDeg);
    }

    for (std::size_t i = 0; i < poses.size(); ++i) {
        MeshCollisionCheckRequest request;
        request.joints = jointsFromDegrees(poses[i]);
        request.returnAllPairs = true;
        const CollisionCheckResult result = CollisionBackends::checkMesh(config, loaded.value, request);
        QCOMPARE(result.status, KinematicsStatus::Ok);

        QString colliding;
        for (const CollisionPairResult& pair : result.pairs) {
            if (pair.colliding) {
                colliding += QString::fromStdString(pair.geometryA + "/" + pair.geometryB + " ");
            }
        }
        QVERIFY2(!result.hasCollision,
                 qPrintable(QString("reference pose %1 collides: %2").arg(int(i)).arg(colliding)));
    }
#endif
}

void NachiMZ07FTests::meshBackendDetectsFoldedSelfCollisionWhenCompiled()
{
#ifndef ROBOTKINEMATICS_HAVE_COAL_MESH_BACKEND
    QSKIP("Coal mesh backend is not compiled in this build");
#else
    const SerialRobotConfig config = Presets::nachiMZ07F();
    const Result<MeshCollisionProfile> loaded =
        MeshCollisionProfileJsonLoader::loadFile(presetFilePath("nachi_mz07f_mesh_collision.json"));
    QVERIFY2(loaded.ok(), loaded.message.c_str());

    // Upper arm pitched forward-down to its J2 limit and forearm curled back to its J3 limit: the
    // wrist is driven into the base/turret. This is a geometric smoke pose inside the joint limits
    // (not an observed controller alarm); it guards that the placed meshes actually meet.
    MeshCollisionCheckRequest request;
    request.joints = jointsFromDegrees({0.0, -45.0, -65.0, 0.0, 0.0, 0.0});
    request.returnAllPairs = true;
    const CollisionCheckResult result = CollisionBackends::checkMesh(config, loaded.value, request);

    QCOMPARE(result.status, KinematicsStatus::Ok);
    QVERIFY2(result.hasCollision, "Folded MZ07F pose should drive the wrist into the base/turret meshes.");

    const auto isAny = [](const std::string& id, std::initializer_list<const char*> ids) {
        return std::any_of(ids.begin(), ids.end(), [&](const char* candidate) { return id == candidate; });
    };
    bool wristHitsBaseOrTurret = false;
    for (const CollisionPairResult& pair : result.pairs) {
        if (!pair.colliding) {
            continue;
        }
        const bool aProximal = isAny(pair.geometryA, {"base_mesh", "j1_mesh"});
        const bool bProximal = isAny(pair.geometryB, {"base_mesh", "j1_mesh"});
        const bool aWrist = isAny(pair.geometryA, {"j4_mesh", "j5_mesh", "j6_mesh"});
        const bool bWrist = isAny(pair.geometryB, {"j4_mesh", "j5_mesh", "j6_mesh"});
        wristHitsBaseOrTurret = wristHitsBaseOrTurret || (aProximal && bWrist) || (bProximal && aWrist);
    }
    QVERIFY(wristHitsBaseOrTurret);
#endif
}

int runNachiMZ07FTests(int argc, char** argv)
{
    NachiMZ07FTests tests;
    return QTest::qExec(&tests, argc, argv);
}
