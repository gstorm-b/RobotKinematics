#include <RobotKinematics/Collision/StlPrimitiveAuthoringHelper.h>

#include <RobotKinematics/Core/Pose.h>

#include <QByteArray>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QString>
#include <QStringList>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace RobotKinematics {

namespace {
constexpr double kPi = 3.14159265358979323846;

struct ParsedMesh {
    StlFileFormat format = StlFileFormat::Ascii;
    std::size_t triangleCount = 0;
    std::vector<Eigen::Vector3d> vertices;
};

struct ProposedGeometryData {
    CollisionGeometry geometry;
    std::array<double, 3> xyz_m = {0.0, 0.0, 0.0};
    std::array<double, 3> rpy_rad = {0.0, 0.0, 0.0};
};

Result<StlPrimitiveProposal> invalidRequest(const std::string& message)
{
    return Result<StlPrimitiveProposal>::failure(KinematicsStatus::InvalidRequest, message);
}

std::uint32_t readLeUInt32(const QByteArray& bytes, int offset)
{
    std::uint32_t value = 0;
    std::memcpy(&value, bytes.constData() + offset, sizeof(value));
    return value;
}

float readLeFloat(const QByteArray& bytes, int offset)
{
    float value = 0.0f;
    std::memcpy(&value, bytes.constData() + offset, sizeof(value));
    return value;
}

bool tryParseBinaryStl(const QByteArray& bytes, ParsedMesh& parsed)
{
    if (bytes.size() < 84) {
        return false;
    }

    const std::uint32_t triangleCount = readLeUInt32(bytes, 80);
    const std::uint64_t expectedSize = 84ull + static_cast<std::uint64_t>(triangleCount) * 50ull;
    if (expectedSize != static_cast<std::uint64_t>(bytes.size())) {
        return false;
    }

    parsed.format = StlFileFormat::Binary;
    parsed.triangleCount = triangleCount;
    parsed.vertices.reserve(static_cast<std::size_t>(triangleCount) * 3);

    int offset = 84;
    for (std::uint32_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
        offset += 12;
        for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            const double x = readLeFloat(bytes, offset);
            const double y = readLeFloat(bytes, offset + 4);
            const double z = readLeFloat(bytes, offset + 8);
            parsed.vertices.emplace_back(x, y, z);
            offset += 12;
        }
        offset += 2;
    }

    return !parsed.vertices.empty();
}

bool tryParseAsciiStl(const QByteArray& bytes, ParsedMesh& parsed)
{
    const QString text = QString::fromUtf8(bytes);
    const QStringList lines = text.split(QRegularExpression("[\r\n]+"), Qt::SkipEmptyParts);

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(static_cast<std::size_t>(lines.size()));

    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (!line.startsWith(QStringLiteral("vertex "))) {
            continue;
        }

        const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() != 4) {
            return false;
        }

        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const double x = parts[1].toDouble(&okX);
        const double y = parts[2].toDouble(&okY);
        const double z = parts[3].toDouble(&okZ);
        if (!okX || !okY || !okZ) {
            return false;
        }

        vertices.emplace_back(x, y, z);
    }

    if (vertices.empty() || (vertices.size() % 3) != 0) {
        return false;
    }

    parsed.format = StlFileFormat::Ascii;
    parsed.triangleCount = vertices.size() / 3;
    parsed.vertices = std::move(vertices);
    return true;
}

std::string defaultIfEmpty(const std::string& value, const std::string& fallback)
{
    return value.empty() ? fallback : value;
}

StlMeshStatistics computeStatistics(const std::vector<Eigen::Vector3d>& vertices, std::size_t triangleCount)
{
    Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d maximum = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    for (const Eigen::Vector3d& vertex : vertices) {
        minimum = minimum.cwiseMin(vertex);
        maximum = maximum.cwiseMax(vertex);
        centroid += vertex;
    }

    centroid /= static_cast<double>(vertices.size());
    const Eigen::Vector3d axisLengths = maximum - minimum;

    StlMeshStatistics statistics;
    statistics.triangleCount = triangleCount;
    statistics.minimumBounds_m = {minimum.x(), minimum.y(), minimum.z()};
    statistics.maximumBounds_m = {maximum.x(), maximum.y(), maximum.z()};
    statistics.centroid_m = {centroid.x(), centroid.y(), centroid.z()};
    statistics.axisLengths_m = {axisLengths.x(), axisLengths.y(), axisLengths.z()};
    return statistics;
}

Eigen::Vector3d boundsCenter(const StlMeshStatistics& statistics)
{
    return Eigen::Vector3d(
        0.5 * (statistics.minimumBounds_m[0] + statistics.maximumBounds_m[0]),
        0.5 * (statistics.minimumBounds_m[1] + statistics.maximumBounds_m[1]),
        0.5 * (statistics.minimumBounds_m[2] + statistics.maximumBounds_m[2]));
}

int longestAxisIndex(const StlMeshStatistics& statistics)
{
    int axisIndex = 0;
    double best = statistics.axisLengths_m[0];
    for (int i = 1; i < 3; ++i) {
        if (statistics.axisLengths_m[static_cast<std::size_t>(i)] > best) {
            best = statistics.axisLengths_m[static_cast<std::size_t>(i)];
            axisIndex = i;
        }
    }
    return axisIndex;
}

std::array<double, 3> rpyForAxisIndex(int axisIndex)
{
    if (axisIndex == 0) {
        return {0.0, 0.5 * kPi, 0.0};
    }
    if (axisIndex == 1) {
        return {-0.5 * kPi, 0.0, 0.0};
    }
    return {0.0, 0.0, 0.0};
}

ProposedGeometryData makeSphereProposal(const StlMeshStatistics& statistics,
                                        const StlPrimitiveProposalRequest& request)
{
    const Eigen::Vector3d center = boundsCenter(statistics);
    const Eigen::Vector3d axisLengths(
        statistics.axisLengths_m[0],
        statistics.axisLengths_m[1],
        statistics.axisLengths_m[2]);

    ProposedGeometryData proposal;
    proposal.xyz_m = {center.x(), center.y(), center.z()};
    proposal.rpy_rad = {0.0, 0.0, 0.0};
    proposal.geometry.id = defaultIfEmpty(request.geometryId, "draft_geometry") + "_sphere";
    proposal.geometry.linkId = defaultIfEmpty(request.linkId, "draft_link");
    proposal.geometry.shape.type = CollisionShapeType::Sphere;
    proposal.geometry.shape.sphere.radius_m = 0.5 * axisLengths.norm();
    proposal.geometry.geometryToLink =
        Pose::fromXYZRPY_m_rad(center.x(), center.y(), center.z(), 0.0, 0.0, 0.0);
    proposal.geometry.margin_m = request.margin_m;
    proposal.geometry.enabled = request.enabled;
    return proposal;
}

ProposedGeometryData makeCapsuleProposal(const StlMeshStatistics& statistics,
                                         const StlPrimitiveProposalRequest& request)
{
    const Eigen::Vector3d center = boundsCenter(statistics);
    const int axisIndex = longestAxisIndex(statistics);

    const double axisA = statistics.axisLengths_m[static_cast<std::size_t>((axisIndex + 1) % 3)];
    const double axisB = statistics.axisLengths_m[static_cast<std::size_t>((axisIndex + 2) % 3)];
    const double longestAxis = statistics.axisLengths_m[static_cast<std::size_t>(axisIndex)];
    const double radius = 0.5 * std::sqrt(axisA * axisA + axisB * axisB);
    const double length = std::max(0.0, longestAxis - 2.0 * radius);
    const std::array<double, 3> rpy = rpyForAxisIndex(axisIndex);

    ProposedGeometryData proposal;
    proposal.xyz_m = {center.x(), center.y(), center.z()};
    proposal.rpy_rad = rpy;
    proposal.geometry.id = defaultIfEmpty(request.geometryId, "draft_geometry") + "_capsule";
    proposal.geometry.linkId = defaultIfEmpty(request.linkId, "draft_link");
    proposal.geometry.shape.type = CollisionShapeType::Capsule;
    proposal.geometry.shape.capsule.radius_m = radius;
    proposal.geometry.shape.capsule.length_m = length;
    proposal.geometry.geometryToLink =
        Pose::fromXYZRPY_m_rad(center.x(), center.y(), center.z(), rpy[0], rpy[1], rpy[2]);
    proposal.geometry.margin_m = request.margin_m;
    proposal.geometry.enabled = request.enabled;
    return proposal;
}

QJsonObject toGeometryJson(const ProposedGeometryData& proposal)
{
    QJsonObject object;
    object["id"] = QString::fromStdString(proposal.geometry.id);
    object["link"] = QString::fromStdString(proposal.geometry.linkId);
    object["enabled"] = proposal.geometry.enabled;
    object["margin_m"] = proposal.geometry.margin_m;

    QJsonObject geometryToLink;
    geometryToLink["xyz_m"] =
        QJsonArray{proposal.xyz_m[0], proposal.xyz_m[1], proposal.xyz_m[2]};
    geometryToLink["rpy_rad"] =
        QJsonArray{proposal.rpy_rad[0], proposal.rpy_rad[1], proposal.rpy_rad[2]};
    object["geometryToLink"] = geometryToLink;

    if (proposal.geometry.shape.type == CollisionShapeType::Sphere) {
        object["shape"] = QStringLiteral("sphere");
        QJsonObject sphere;
        sphere["radius_m"] = proposal.geometry.shape.sphere.radius_m;
        object["sphere"] = sphere;
    } else {
        object["shape"] = QStringLiteral("capsule");
        QJsonObject capsule;
        capsule["radius_m"] = proposal.geometry.shape.capsule.radius_m;
        capsule["length_m"] = proposal.geometry.shape.capsule.length_m;
        object["capsule"] = capsule;
    }

    return object;
}

std::string draftJsonFor(const ProposedGeometryData& proposal,
                         const StlPrimitiveProposalRequest& request,
                         const char* primitiveKind)
{
    QJsonObject root;
    root["schema"] = QStringLiteral("robot-kinematics-collision/v1");

    QJsonObject profile;
    profile["id"] = QString::fromStdString(defaultIfEmpty(request.profileId, "draft_collision_profile"));
    profile["robotModel"] = QString::fromStdString(defaultIfEmpty(request.robotModel, "draft_robot"));
    QJsonObject units;
    units["length"] = QStringLiteral("m");
    units["angle"] = QStringLiteral("rad");
    profile["units"] = units;
    root["profile"] = profile;

    root["geometries"] = QJsonArray{toGeometryJson(proposal)};
    root["disabledPairs"] = QJsonArray{};

    QJsonObject source;
    source["type"] = QStringLiteral("stl_authoring_helper");
    source["title"] = QStringLiteral("Draft primitive proposal from STL bounds");
    source["reference"] = QStringLiteral("draft_manual_review_required");
    source["appliesTo"] = QJsonArray{QStringLiteral("collision_geometry")};
    root["sources"] = QJsonArray{source};

    QJsonObject metadata;
    metadata["reviewState"] = QStringLiteral("draft_manual_review_required");
    metadata["helper"] = QStringLiteral("StlPrimitiveAuthoringHelper");
    metadata["primitive"] = QString::fromLatin1(primitiveKind);
    root["metadata"] = metadata;

    return QJsonDocument(root).toJson(QJsonDocument::Indented).toStdString();
}
}

Result<StlPrimitiveProposal> StlPrimitiveAuthoringHelper::proposeFromFile(
    const std::string& path,
    const StlPrimitiveProposalRequest& request)
{
    if (request.margin_m < 0.0) {
        return invalidRequest("STL primitive proposal margin must be non-negative");
    }

    QFile file(QString::fromStdString(path));
    if (!file.open(QIODevice::ReadOnly)) {
        return invalidRequest("STL file could not be opened");
    }

    const QByteArray bytes = file.readAll();
    ParsedMesh parsed;
    if (!tryParseBinaryStl(bytes, parsed) && !tryParseAsciiStl(bytes, parsed)) {
        return invalidRequest("STL payload is not a supported ASCII or binary STL");
    }

    const StlMeshStatistics statistics = computeStatistics(parsed.vertices, parsed.triangleCount);
    const ProposedGeometryData sphere = makeSphereProposal(statistics, request);
    const ProposedGeometryData capsule = makeCapsuleProposal(statistics, request);

    StlPrimitiveProposal proposal;
    proposal.format = parsed.format;
    proposal.statistics = statistics;
    proposal.sphere = sphere.geometry;
    proposal.capsule = capsule.geometry;
    proposal.sphereDraftJson = draftJsonFor(sphere, request, "sphere");
    proposal.capsuleDraftJson = draftJsonFor(capsule, request, "capsule");
    proposal.reviewNote = "draft_manual_review_required";

    return Result<StlPrimitiveProposal>::success(proposal);
}

} // namespace RobotKinematics
