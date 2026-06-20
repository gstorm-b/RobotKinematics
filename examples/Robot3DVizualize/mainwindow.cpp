#include "mainwindow.h"
#include "Robot3DVisualizerLogic.h"
#include "ui_mainwindow.h"

#include <QCoreApplication>
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QHeaderView>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStringList>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>

#include <RobotKinematics/Core/Units.h>
#include <RobotKinematics/Kinematics/JointLimitValidator.h>
#include <RobotKinematics/Presets/NachiMZ04D.h>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkLight.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>

#include <array>
#include <optional>

using namespace RobotKinematics;

namespace
{
struct RobotVisualPartSpec
{
    const char* key;
    const char* displayName;
    const char* fileName;
    const char* colorName;
    const char* linkId;
};

constexpr std::array<RobotVisualPartSpec, 8> kRobotParts = {{
    {"base", "Base", "MZ04-01_base.stl", "Silver", "base_link"},
    {"j1", "Joint 1", "MZ04-01_j1.stl", "SlateGray", "link_1"},
    {"j2", "Joint 2", "MZ04-01_j2.stl", "LightSteelBlue", "link_2"},
    {"j3", "Joint 3", "MZ04-01_j3.stl", "CadetBlue", "link_3"},
    {"j4", "Joint 4", "MZ04-01_j4.stl", "LightSkyBlue", "link_4"},
    {"j5", "Joint 5", "MZ04-01_j5.stl", "SteelBlue", "link_5"},
    {"j6", "Joint 6", "MZ04-01_j6.stl", "DodgerBlue", "flange"},
    {"tool", "Centering Tool Mesh", "Centering_tool.stl", "Tomato", "flange"},
}};

constexpr std::array<double, 6> kMidPointDegrees = {
    0.0, 90.0, 0.0, 0.0, 0.0, 0.0,
};


constexpr std::array<double, 6> kTeachPoint1Degrees = {
    28.1579, -18.8069, 163.839, -0.710019, 35.8922, 152.731,
};

constexpr std::array<double, 6> kTeachPoint20Degrees = {
    -0.00219726, 0.00430813, 179.996, 0.00459559, 0.0046875, -0.000121055,
};

QString findAssetsDirectory()
{
    const QString appDirPath = QCoreApplication::applicationDirPath();

    QStringList candidateRoots;
    candidateRoots << appDirPath << QDir::currentPath();

    QDir searchDir(appDirPath);
    for (int depth = 0; depth < 8; ++depth) {
        candidateRoots << searchDir.absolutePath();
        if (!searchDir.cdUp()) {
            break;
        }
    }

    for (const QString& rootPath : candidateRoots) {
        const QDir rootDir(rootPath);
        const QString directAssets = rootDir.filePath(QStringLiteral("3d_model"));
        if (QDir(directAssets).exists()) {
            return QDir(directAssets).absolutePath();
        }

        const QString repoAssets =
            rootDir.filePath(QStringLiteral("examples/Robot3DVizualize/3d_model"));
        if (QDir(repoAssets).exists()) {
            return QDir(repoAssets).absolutePath();
        }
    }

    return QString();
}

QString formatNumber(double value, int decimals)
{
    return QString::number(value, 'f', decimals);
}

std::array<double, 6> midpointDegrees(const SerialRobotConfig& config)
{
    std::array<double, 6> midpoint{};
    int movableIndex = 0;
    for (const Joint& joint : config.joints) {
        if (joint.type != JointType::Revolute && joint.type != JointType::Prismatic) {
            continue;
        }

        double value = 0.0;
        if (joint.limits.has_value()) {
            value = 0.5 * (joint.limits->lower + joint.limits->upper);
        } else {
            value = joint.home;
        }
        midpoint[static_cast<std::size_t>(movableIndex)] = units::toDeg(value);
        ++movableIndex;
    }
    return midpoint;
}

std::array<double, 6> homeDegrees(const SerialRobotConfig& config)
{
    std::array<double, 6> home{};
    int movableIndex = 0;
    for (const Joint& joint : config.joints) {
        if (joint.type != JointType::Revolute && joint.type != JointType::Prismatic) {
            continue;
        }

        home[static_cast<std::size_t>(movableIndex)] = units::toDeg(joint.home);
        ++movableIndex;
    }
    return home;
}

Pose poseForLinkId(const FkChain& chain, const std::string& linkId)
{
    const auto it = chain.linkPosesInBase.find(linkId);
    return it == chain.linkPosesInBase.end() ? Pose::identity() : it->second;
}

int jointAxisIndexForPartKey(const QString& key)
{
    if (key == QStringLiteral("j1")) {
        return 0;
    } else if (key == QStringLiteral("j2")) {
        return 1;
    } else if (key == QStringLiteral("j3")) {
        return 2;
    } else if (key == QStringLiteral("j4")) {
        return 3;
    } else if (key == QStringLiteral("j5")) {
        return 4;
    } else if (key == QStringLiteral("j6")) {
        return 5;
    }
    return -1;
}

Pose visualHomeCorrectionForPartKey(const QString& key)
{
    // Placement tuning lives here. When a mesh was exported in a CAD frame that does not
    // match the canonical link frame, edit the per-part correction in this function.
    // The returned pose is multiplied after the FK home-relative delta in updateSceneFromChain().
    if (key == QStringLiteral("base")) {
        return Pose::fromXYZRPY_mm_deg(0.0, 0.0, 0.0, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j1")) {
        return Pose::fromXYZRPY_mm_deg(0.0, 0.0, 340.0, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j2")) {
        return Pose::fromXYZRPY_mm_deg(0.0, 0.0, 340.0, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j3")) {
        return Pose::fromXYZRPY_mm_deg(260.0, 0.0, 340, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j4")) {
        return Pose::fromXYZRPY_mm_deg(285.0, 0.0, 259, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j5")) {
        return Pose::fromXYZRPY_mm_deg(285.0, 0.0, 60, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("j6")) {
        return Pose::fromXYZRPY_mm_deg(285.0, 0.0, -12, 90.0, 0.0, 0.0);
    } else if (key == QStringLiteral("tool")) {
        return Pose::fromXYZRPY_mm_deg(285.0, 0.0, -12, 0.0, 0.0, 180.0);
    }

    return Pose::identity();
}

SerialRobotConfig buildExampleConfig()
{
    SerialRobotConfig config = Presets::nachiMZ04D();
    config.tools.push_back(Tool{
        "centering_tool",
        "Centering Tool (45, 0, 112 mm)",
        Pose::fromXYZRPY_mm_deg(45.0, 0.0, 112.0, 0.0, 0.0, 0.0),
    });
    config.metadata["example_visual_tool"] = "centering_tool";
    return config;
}
} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , config_(buildExampleConfig())
    , robot_(config_)
    , frameRegistry_(FrameRegistry::fromConfig(config_))
    , toolRegistry_(ToolRegistry::fromConfig(config_))
    , postureResolver_(PostureResolverFactory::create(config_))
{
    ui->setupUi(this);

    setWindowTitle(QStringLiteral("RobotKinematics - Nachi MZ04D Pose Visualizer"));

    setupVtkViewport();
    setupModelState();
    setupUiState();
    connectSignals();
    loadRobotVisuals();

    setJointDegrees(homeDegrees(config_));
    resetTargetToCurrentTcp();
    applyJointStateToSceneAndReadouts();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupVtkViewport()
{
    auto* hostLayout = new QVBoxLayout(ui->vtkContainerWidget);
    hostLayout->setContentsMargins(0, 0, 0, 0);
    hostLayout->setSpacing(0);

    vtkWidget_ = new QVTKOpenGLNativeWidget(ui->vtkContainerWidget);
    hostLayout->addWidget(vtkWidget_);

    renderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer_ = vtkSmartPointer<vtkRenderer>::New();

    vtkWidget_->setRenderWindow(renderWindow_);
    renderWindow_->AddRenderer(renderer_);
    renderer_->SetBackground(0.08, 0.09, 0.11);
    renderer_->SetBackground2(0.18, 0.20, 0.24);
    renderer_->GradientBackgroundOn();

    vtkSmartPointer<vtkLight> keyLight = vtkSmartPointer<vtkLight>::New();
    keyLight->SetLightTypeToCameraLight();
    keyLight->SetPosition(0.3, 0.4, 1.0);
    keyLight->SetFocalPoint(0.0, 0.0, 0.0);
    keyLight->SetIntensity(1.1);
    renderer_->AddLight(keyLight);

    vtkSmartPointer<vtkPlaneSource> groundPlane = vtkSmartPointer<vtkPlaneSource>::New();
    groundPlane->SetOrigin(-500.0, -500.0, -5.0);
    groundPlane->SetPoint1(500.0, -500.0, -5.0);
    groundPlane->SetPoint2(-500.0, 500.0, -5.0);
    groundPlane->SetXResolution(10);
    groundPlane->SetYResolution(10);

    vtkSmartPointer<vtkPolyDataMapper> groundMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    groundMapper->SetInputConnection(groundPlane->GetOutputPort());

    vtkSmartPointer<vtkActor> groundActor = vtkSmartPointer<vtkActor>::New();
    groundActor->SetMapper(groundMapper);
    groundActor->GetProperty()->SetColor(0.32, 0.36, 0.40);
    groundActor->GetProperty()->SetRepresentationToWireframe();
    groundActor->GetProperty()->SetOpacity(0.55);
    renderer_->AddActor(groundActor);

    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    axesActor->SetTotalLength(120.0, 120.0, 120.0);
    orientationMarker_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientationMarker_->SetOrientationMarker(axesActor);
    orientationMarker_->SetViewport(0.0, 0.0, 0.2, 0.2);
    orientationMarker_->SetInteractor(renderWindow_->GetInteractor());
    orientationMarker_->SetEnabled(1);
    orientationMarker_->InteractiveOff();
}

void MainWindow::setupModelState()
{
    if (!postureResolver_) {
        QMessageBox::warning(this,
                             QStringLiteral("Posture metadata unavailable"),
                             QStringLiteral("The Nachi posture resolver could not be created. "
                                            "Posture controls will remain disabled."));
    }
}

void MainWindow::setupUiState()
{
    ui->mainSplitter->setStretchFactor(0, 0);
    ui->mainSplitter->setStretchFactor(1, 1);

    populateCombos();
    populateJointControls();
    populatePostureControls();
    populateSampleButtons();
    populateDebugControls();

    ui->ikResultsTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->ikResultsTableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->ikResultsTableWidget->verticalHeader()->setVisible(false);

    ui->sceneTitleLabel->setText(QStringLiteral("VTK Scene (visual world in CAD millimeters)"));
    ui->jointStatusLabel->setText(QStringLiteral("Joint state not evaluated yet."));
    ui->ikStatusLabel->setText(QStringLiteral("IK status will appear here."));

    updateActionState();
}

void MainWindow::connectSignals()
{
    for (QDoubleSpinBox* spinBox : jointSpinBoxes()) {
        connect(spinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                [this](double) { applyJointStateToSceneAndReadouts(); });
    }

    connect(ui->toolComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { applyJointStateToSceneAndReadouts(); });
    connect(ui->referenceFrameComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { applyJointStateToSceneAndReadouts(); });

    connect(ui->homeButton, &QPushButton::clicked, this,
            [this]() { setJointDegrees(homeDegrees(config_)); });
    connect(ui->midpointButton, &QPushButton::clicked, this,
            [this]() { setJointDegrees(kMidPointDegrees); });
    connect(ui->teachPoint1Button, &QPushButton::clicked, this,
            [this]() { setJointDegrees(kTeachPoint1Degrees); });
    connect(ui->teachPoint20Button, &QPushButton::clicked, this,
            [this]() { setJointDegrees(kTeachPoint20Degrees); });

    connect(ui->copyCurrentTcpToTargetButton, &QPushButton::clicked, this, &MainWindow::resetTargetToCurrentTcp);
    connect(ui->solveBestButton, &QPushButton::clicked, this,
            [this]() { solveInverseKinematics(false); });
    connect(ui->solveAllButton, &QPushButton::clicked, this,
            [this]() { solveInverseKinematics(true); });
    connect(ui->applySelectedSolutionButton, &QPushButton::clicked, this, &MainWindow::applySelectedIkSolution);
    connect(ui->ikResultsTableWidget, &QTableWidget::itemSelectionChanged, this, &MainWindow::updateActionState);

    for (QCheckBox* checkBox : partVisibleCheckBoxes()) {
        connect(checkBox, &QCheckBox::toggled, this, [this](bool) { applyDebugVisualState(); });
    }
    for (QCheckBox* checkBox : partOriginCheckBoxes()) {
        connect(checkBox, &QCheckBox::toggled, this, [this](bool) { applyDebugVisualState(); });
    }
    for (QCheckBox* checkBox : partAxisCheckBoxes()) {
        connect(checkBox, &QCheckBox::toggled, this, [this](bool) { applyDebugVisualState(); });
    }
}

void MainWindow::populateCombos()
{
    ui->toolComboBox->clear();
    for (const Tool& tool : config_.tools) {
        ui->toolComboBox->addItem(QString::fromStdString(tool.name), QString::fromStdString(tool.id));
    }

    const int centeringToolIndex = ui->toolComboBox->findData(QStringLiteral("centering_tool"));
    if (centeringToolIndex >= 0) {
        ui->toolComboBox->setCurrentIndex(centeringToolIndex);
    } else {
        const int defaultIndex =
            ui->toolComboBox->findData(QString::fromStdString(config_.defaultToolId));
        ui->toolComboBox->setCurrentIndex(defaultIndex >= 0 ? defaultIndex : 0);
    }

    ui->referenceFrameComboBox->clear();
    ui->referenceFrameComboBox->addItem(QStringLiteral("Base"), QStringLiteral("base"));
    for (const UserFrame& frame : config_.frames.userFrames) {
        ui->referenceFrameComboBox->addItem(QString::fromStdString(frame.id),
                                            QString::fromStdString(frame.id));
    }
}

void MainWindow::populateJointControls()
{
    int movableIndex = 0;
    for (const Joint& joint : config_.joints) {
        if (joint.type != JointType::Revolute && joint.type != JointType::Prismatic) {
            continue;
        }

        QDoubleSpinBox* spinBox = jointSpinBoxes().at(static_cast<std::size_t>(movableIndex));
        spinBox->setSuffix(QStringLiteral(" deg"));
        if (joint.limits.has_value()) {
            spinBox->setRange(units::toDeg(joint.limits->lower), units::toDeg(joint.limits->upper));
        } else {
            spinBox->setRange(-360.0, 360.0);
        }
        ++movableIndex;
    }

    ui->targetXSpinBox->setSuffix(QStringLiteral(" mm"));
    ui->targetYSpinBox->setSuffix(QStringLiteral(" mm"));
    ui->targetZSpinBox->setSuffix(QStringLiteral(" mm"));
    ui->targetRzSpinBox->setSuffix(QStringLiteral(" deg"));
    ui->targetRySpinBox->setSuffix(QStringLiteral(" deg"));
    ui->targetRxSpinBox->setSuffix(QStringLiteral(" deg"));
}

void MainWindow::populatePostureControls()
{
    const auto addBranchItems = [&](QComboBox* comboBox, const char* axis) {
        comboBox->clear();
        comboBox->addItem(QStringLiteral("Any"), QString());

        const auto it = config_.posture.labels.find(axis);
        if (it == config_.posture.labels.end()) {
            return;
        }

        comboBox->addItem(QString::fromStdString(it->second.negative),
                          QString::fromStdString(it->second.negative));
        comboBox->addItem(QString::fromStdString(it->second.positive),
                          QString::fromStdString(it->second.positive));
    };

    addBranchItems(ui->shoulderRequestComboBox, "shoulder");
    addBranchItems(ui->elbowRequestComboBox, "elbow");
    addBranchItems(ui->wristRequestComboBox, "wrist");
}

void MainWindow::populateSampleButtons()
{
    ui->teachPoint1Button->setToolTip(
        QStringLiteral("Teach-pendant measurement point 1 from docs/preset_references/nachi-mz04d.md"));
    ui->teachPoint20Button->setToolTip(
        QStringLiteral("Teach-pendant measurement point 20 from docs/preset_references/nachi-mz04d.md"));
}

void MainWindow::populateDebugControls()
{
    const std::array<QCheckBox*, 8> visible = partVisibleCheckBoxes();
    const std::array<QCheckBox*, 8> origins = partOriginCheckBoxes();
    const std::array<QCheckBox*, 8> axes = partAxisCheckBoxes();

    for (QCheckBox* checkBox : visible) {
        checkBox->setChecked(true);
    }
    for (QCheckBox* checkBox : origins) {
        checkBox->setChecked(false);
    }
    for (QCheckBox* checkBox : axes) {
        checkBox->setChecked(false);
    }

    ui->baseAxisCheckBox->setEnabled(false);
    ui->toolAxisCheckBox->setEnabled(false);

    ui->debugHintLabel->setText(
        QStringLiteral("Use these toggles to inspect where each STL local origin ended up in the "
                       "scene. Placement parameters live in mainwindow.cpp: "
                       "`visualHomeCorrectionForPartKey()` for per-mesh corrections and "
                       "`buildExampleConfig()` for the tool TCP offset. The Axis column shows "
                       "the actual FK joint axis from `chain.joints`."));
}

void MainWindow::loadRobotVisuals()
{
    assetsDirectory_ = findAssetsDirectory();
    QStringList loadErrors;

    if (assetsDirectory_.isEmpty()) {
        loadErrors << QStringLiteral(
            "Could not find the `3d_model` runtime asset directory. Build from the repository "
            "root or keep the example next to `examples/Robot3DVizualize/3d_model`.");
    } else {
        const JointVector homeJoints = JointVector::fromDegrees(
            {homeDegrees(config_)[0], homeDegrees(config_)[1], homeDegrees(config_)[2],
             homeDegrees(config_)[3], homeDegrees(config_)[4], homeDegrees(config_)[5]});
        const FkChain homeChain = ForwardKinematics::computeChain(config_, homeJoints);
        vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

        for (const RobotVisualPartSpec& spec : kRobotParts) {
            const QString meshPath = QDir(assetsDirectory_).filePath(QString::fromLatin1(spec.fileName));
            QFileInfo meshInfo(meshPath);
            if (!meshInfo.exists() || !meshInfo.isFile()) {
                loadErrors << QStringLiteral("%1 mesh is missing: %2")
                                  .arg(QString::fromLatin1(spec.displayName), meshPath);
                continue;
            }

            vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(meshInfo.absoluteFilePath().toLocal8Bit().constData());
            reader->Update();

            if (!reader->GetOutput() || reader->GetOutput()->GetNumberOfPoints() == 0) {
                loadErrors << QStringLiteral("%1 mesh is unreadable or empty: %2")
                                  .arg(QString::fromLatin1(spec.displayName), meshPath);
                continue;
            }

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(reader->GetOutputPort());

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetColor(colors->GetColor3d(spec.colorName).GetData());
            actor->GetProperty()->SetInterpolationToPhong();
            actor->GetProperty()->SetSpecular(0.18);
            actor->GetProperty()->SetSpecularPower(18.0);
            renderer_->AddActor(actor);

            vtkSmartPointer<vtkAxesActor> originActor = vtkSmartPointer<vtkAxesActor>::New();
            originActor->SetTotalLength(60.0, 60.0, 60.0);
            originActor->SetShaftTypeToLine();
            originActor->AxisLabelsOff();
            renderer_->AddActor(originActor);

            vtkSmartPointer<vtkLineSource> axisSource = vtkSmartPointer<vtkLineSource>::New();
            axisSource->SetPoint1(0.0, 0.0, 0.0);
            axisSource->SetPoint2(0.0, 0.0, 100.0);

            vtkSmartPointer<vtkPolyDataMapper> axisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            axisMapper->SetInputConnection(axisSource->GetOutputPort());

            vtkSmartPointer<vtkActor> axisActor = vtkSmartPointer<vtkActor>::New();
            axisActor->SetMapper(axisMapper);
            axisActor->GetProperty()->SetColor(1.0, 0.85, 0.1);
            axisActor->GetProperty()->SetLineWidth(3.0);
            renderer_->AddActor(axisActor);

            VisualPartState state;
            state.key = QString::fromLatin1(spec.key);
            state.displayName = QString::fromLatin1(spec.displayName);
            state.linkId = spec.linkId;
            state.homeLinkInBase = poseForLinkId(homeChain, spec.linkId);
            state.homeVisualCorrection = visualHomeCorrectionForPartKey(state.key);
            state.jointAxisIndex = jointAxisIndexForPartKey(state.key);
            state.isLoaded = true;
            state.actor = actor;
            state.originActor = originActor;
            state.axisSource = axisSource;
            state.axisActor = axisActor;
            visualParts_.push_back(state);
        }
    }

    applyDebugVisualState();

    renderer_->ResetCamera();
    renderWindow_->Render();

    if (loadErrors.isEmpty()) {
        statusBar()->showMessage(
            QStringLiteral("Loaded %1 robot meshes from %2")
                .arg(visualParts_.size())
                .arg(QDir::toNativeSeparators(assetsDirectory_)));
    } else {
        const QString errorText = loadErrors.join(QStringLiteral("\n"));
        statusBar()->showMessage(QStringLiteral("Asset loading completed with warnings"));
        QMessageBox::warning(
            this,
            QStringLiteral("Robot assets missing"),
            QStringLiteral("Some STL assets could not be loaded:\n\n%1").arg(errorText));
    }
}

void MainWindow::applyJointStateToSceneAndReadouts()
{
    const JointVector joints = currentJointVector();
    const FkChain chain = ForwardKinematics::computeChain(config_, joints);

    updateSceneFromChain(chain);
    updatePoseReadouts(chain);
    updateJointStatus(joints);
    updateCurrentPosture(joints);
    updateActionState();

    renderWindow_->Render();
}

void MainWindow::updateSceneFromChain(const FkChain& chain)
{
    for (VisualPartState& part : visualParts_) {
        if (!part.actor) {
            continue;
        }

        // This is the final place where each mesh's VTK transform is set.
        // If a specific exported STL is offset or rotated incorrectly, adjust
        // its parameters in visualHomeCorrectionForPartKey() above.
        const Pose currentLinkPose = poseForLinkId(chain, part.linkId);
        const Eigen::Matrix4d matrixValues =
            Robot3DVisualizer::visualDeltaMatrixMm(
                currentLinkPose, part.homeLinkInBase, part.homeVisualCorrection);

        vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
        for (int row = 0; row < 4; ++row) {
            for (int column = 0; column < 4; ++column) {
                matrix->SetElement(row, column, matrixValues(row, column));
            }
        }
        part.actor->SetUserMatrix(matrix);
        if (part.originActor) {
            part.originActor->SetUserMatrix(matrix);
        }
        if (part.axisActor && part.axisSource && part.jointAxisIndex >= 0 &&
            part.jointAxisIndex < static_cast<int>(chain.joints.size())) {
            const JointFrameData& jointData = chain.joints[static_cast<std::size_t>(part.jointAxisIndex)];
            const Eigen::Vector3d originMm = jointData.originInBase * 1000.0;
            const Eigen::Vector3d axisMm = jointData.axisInBase.normalized() * 120.0;
            const Eigen::Vector3d point1 = originMm - axisMm;
            const Eigen::Vector3d point2 = originMm + axisMm;
            part.axisSource->SetPoint1(point1.x(), point1.y(), point1.z());
            part.axisSource->SetPoint2(point2.x(), point2.y(), point2.z());
            part.axisSource->Update();
        }
    }

    applyDebugVisualState();
}

void MainWindow::applyDebugVisualState()
{
    const std::array<QCheckBox*, 8> visible = partVisibleCheckBoxes();
    const std::array<QCheckBox*, 8> origins = partOriginCheckBoxes();
    const std::array<QCheckBox*, 8> axes = partAxisCheckBoxes();

    for (std::size_t index = 0; index < visualParts_.size() && index < visible.size(); ++index) {
        VisualPartState& part = visualParts_[index];
        if (part.actor) {
            part.actor->SetVisibility(part.isLoaded && visible[index]->isChecked());
        }
        if (part.originActor) {
            part.originActor->SetVisibility(part.isLoaded && origins[index]->isChecked());
        }
        if (part.axisActor) {
            const bool hasAxis = part.jointAxisIndex >= 0;
            part.axisActor->SetVisibility(part.isLoaded && hasAxis && axes[index]->isChecked());
        }
    }

    if (renderWindow_) {
        renderWindow_->Render();
    }
}

void MainWindow::updatePoseReadouts(const FkChain& chain)
{
    const Result<Tool> tool = selectedTool();
    const Result<Pose> referenceInBase = selectedReferenceInBase(chain);

    if (!tool.ok()) {
        ui->flangeXLineEdit->setText(QStringLiteral("n/a"));
        ui->tcpXLineEdit->setText(QStringLiteral("n/a"));
        updateIkStatus(QStringLiteral("%1: %2").arg(Robot3DVisualizer::statusText(tool.status), QString::fromStdString(tool.message)));
        return;
    }
    if (!referenceInBase.ok()) {
        updateIkStatus(QStringLiteral("%1: %2")
                           .arg(Robot3DVisualizer::statusText(referenceInBase.status),
                                QString::fromStdString(referenceInBase.message)));
        return;
    }

    const JointVector joints = currentJointVector();
    const Pose flangeInBase = chain.flangeInBase;
    const Pose tcpInBase = ForwardKinematics::toolPose(config_, joints, tool.value.flangeToTcp);
    const Pose flangeInReference = referenceInBase.value.inverse() * flangeInBase;
    const Pose tcpInReference = referenceInBase.value.inverse() * tcpInBase;

    const Robot3DVisualizer::PendantPoseDisplay flangeDisplay =
        Robot3DVisualizer::toNachiPendantPose(flangeInReference);
    const Robot3DVisualizer::PendantPoseDisplay tcpDisplay =
        Robot3DVisualizer::toNachiPendantPose(tcpInReference);

    ui->flangeXLineEdit->setText(formatNumber(flangeDisplay.x_mm, 3));
    ui->flangeYLineEdit->setText(formatNumber(flangeDisplay.y_mm, 3));
    ui->flangeZLineEdit->setText(formatNumber(flangeDisplay.z_mm, 3));
    ui->flangeRzLineEdit->setText(formatNumber(flangeDisplay.rz_deg, 4));
    ui->flangeRyLineEdit->setText(formatNumber(flangeDisplay.ry_deg, 4));
    ui->flangeRxLineEdit->setText(formatNumber(flangeDisplay.rx_deg, 4));

    ui->tcpXLineEdit->setText(formatNumber(tcpDisplay.x_mm, 3));
    ui->tcpYLineEdit->setText(formatNumber(tcpDisplay.y_mm, 3));
    ui->tcpZLineEdit->setText(formatNumber(tcpDisplay.z_mm, 3));
    ui->tcpRzLineEdit->setText(formatNumber(tcpDisplay.rz_deg, 4));
    ui->tcpRyLineEdit->setText(formatNumber(tcpDisplay.ry_deg, 4));
    ui->tcpRxLineEdit->setText(formatNumber(tcpDisplay.rx_deg, 4));
}

void MainWindow::updateJointStatus(const JointVector& joints)
{
    const JointLimitCheck check = JointLimitValidator::validate(config_, joints);
    if (check.ok()) {
        ui->jointStatusLabel->setText(QStringLiteral("Within configured joint limits."));
        return;
    }

    if (!check.violations.empty()) {
        const JointLimitViolation& violation = check.violations.front();
        ui->jointStatusLabel->setText(
            QStringLiteral("%1: %2 = %3 deg is outside [%4, %5] deg")
                .arg(Robot3DVisualizer::statusText(check.status))
                .arg(QString::fromStdString(violation.jointId))
                .arg(formatNumber(units::toDeg(violation.value), 4))
                .arg(formatNumber(units::toDeg(violation.lower), 4))
                .arg(formatNumber(units::toDeg(violation.upper), 4)));
        return;
    }

    ui->jointStatusLabel->setText(Robot3DVisualizer::statusText(check.status));
}

void MainWindow::updateCurrentPosture(const JointVector& joints)
{
    if (!postureResolver_) {
        ui->currentShoulderValueLabel->setText(QStringLiteral("Unavailable"));
        ui->currentElbowValueLabel->setText(QStringLiteral("Unavailable"));
        ui->currentWristValueLabel->setText(QStringLiteral("Unavailable"));
        return;
    }

    const Result<ArmPosture> posture = postureResolver_->classify(config_, joints);
    if (!posture.ok()) {
        ui->currentShoulderValueLabel->setText(QStringLiteral("Unavailable"));
        ui->currentElbowValueLabel->setText(QStringLiteral("Unavailable"));
        ui->currentWristValueLabel->setText(QStringLiteral("Unavailable"));
        return;
    }

    ui->currentShoulderValueLabel->setText(
        Robot3DVisualizer::postureLabel(config_.posture, "shoulder", posture.value.shoulder));
    ui->currentElbowValueLabel->setText(
        Robot3DVisualizer::postureLabel(config_.posture, "elbow", posture.value.elbow));
    ui->currentWristValueLabel->setText(
        Robot3DVisualizer::postureLabel(config_.posture, "wrist", posture.value.wrist));
}

void MainWindow::updateIkStatus(const QString& message)
{
    ui->ikStatusLabel->setText(message);
    statusBar()->showMessage(message);
}

void MainWindow::updateActionState()
{
    const int currentRow = ui->ikResultsTableWidget->currentRow();
    const bool hasSelection = currentRow >= 0 && currentRow < static_cast<int>(lastIkSolutions_.size());
    ui->applySelectedSolutionButton->setEnabled(hasSelection);
}

void MainWindow::resetTargetToCurrentTcp()
{
    const JointVector joints = currentJointVector();
    const FkChain chain = ForwardKinematics::computeChain(config_, joints);
    const Result<Tool> tool = selectedTool();
    const Result<Pose> referenceInBase = selectedReferenceInBase(chain);

    if (!tool.ok()) {
        updateIkStatus(QStringLiteral("%1: %2").arg(Robot3DVisualizer::statusText(tool.status),
                                                    QString::fromStdString(tool.message)));
        return;
    }
    if (!referenceInBase.ok()) {
        updateIkStatus(QStringLiteral("%1: %2")
                           .arg(Robot3DVisualizer::statusText(referenceInBase.status),
                                QString::fromStdString(referenceInBase.message)));
        return;
    }

    const Pose tcpInBase = ForwardKinematics::toolPose(config_, joints, tool.value.flangeToTcp);
    const Pose tcpInReference = referenceInBase.value.inverse() * tcpInBase;
    const Robot3DVisualizer::PendantPoseDisplay display =
        Robot3DVisualizer::toNachiPendantPose(tcpInReference);

    const std::array<QDoubleSpinBox*, 6> spins = targetSpinBoxes();
    const std::array<double, 6> values = {
        display.x_mm, display.y_mm, display.z_mm, display.rz_deg, display.ry_deg, display.rx_deg,
    };
    for (std::size_t i = 0; i < spins.size(); ++i) {
        const QSignalBlocker blocker(spins[i]);
        spins[i]->setValue(values[i]);
    }

    updateIkStatus(QStringLiteral("Copied current TCP pose into the IK target fields."));
}

void MainWindow::solveInverseKinematics(bool solveAll)
{
    const Result<std::optional<ArmPosture>> postureRequest = requestedPosture();
    if (!postureRequest.ok()) {
        updateIkStatus(QStringLiteral("%1: %2")
                           .arg(Robot3DVisualizer::statusText(postureRequest.status),
                                QString::fromStdString(postureRequest.message)));
        return;
    }

    if (ui->requirePostureCheckBox->isChecked() && !postureRequest.value.has_value()) {
        updateIkStatus(QStringLiteral("Select at least one posture branch or disable require posture."));
        return;
    }

    IKRequest request;
    request.targetPose = Robot3DVisualizer::fromNachiPendantPose(
        ui->targetXSpinBox->value(),
        ui->targetYSpinBox->value(),
        ui->targetZSpinBox->value(),
        ui->targetRzSpinBox->value(),
        ui->targetRySpinBox->value(),
        ui->targetRxSpinBox->value());
    request.seedJoint = currentJointVector();
    request.previousJoint = currentJointVector();
    if (postureRequest.value.has_value()) {
        request.posture = *postureRequest.value;
    }
    request.referenceFrame = FrameId{ui->referenceFrameComboBox->currentData().toString().toStdString()};
    request.tool = ToolId{ui->toolComboBox->currentData().toString().toStdString()};
    request.options.requirePosture = ui->requirePostureCheckBox->isChecked();
    request.options.maxSolutions = ui->maxSolutionsSpinBox->value();

    const IKResult result = solveAll ? robot_.solveAll(request) : robot_.solve(request);
    populateIkResults(result);

    QString summary = QStringLiteral("%1 returned %2 solution(s) with status %3")
                          .arg(solveAll ? QStringLiteral("solveAll") : QStringLiteral("solve"))
                          .arg(result.solutions.size())
                          .arg(Robot3DVisualizer::statusText(result.status));
    if (!result.message.empty()) {
        summary += QStringLiteral(": ") + QString::fromStdString(result.message);
    }
    updateIkStatus(summary);
}

void MainWindow::populateIkResults(const IKResult& result)
{
    lastIkSolutions_ = result.solutions;
    ui->ikResultsTableWidget->setRowCount(static_cast<int>(result.solutions.size()));

    for (int row = 0; row < static_cast<int>(result.solutions.size()); ++row) {
        const IKSolution& solution = result.solutions[static_cast<std::size_t>(row)];
        const std::vector<double> jointsDeg = solution.joints.toDegrees();

        auto* rankItem = new QTableWidgetItem(QString::number(row + 1));
        ui->ikResultsTableWidget->setItem(row, 0, rankItem);

        for (int jointIndex = 0; jointIndex < 6; ++jointIndex) {
            auto* jointItem = new QTableWidgetItem(formatNumber(jointsDeg[static_cast<std::size_t>(jointIndex)], 4));
            ui->ikResultsTableWidget->setItem(row, jointIndex + 1, jointItem);
        }

        ui->ikResultsTableWidget->setItem(
            row, 7, new QTableWidgetItem(formatNumber(units::toMm(solution.positionError_m), 6)));
        ui->ikResultsTableWidget->setItem(
            row, 8, new QTableWidgetItem(formatNumber(units::toDeg(solution.orientationError_rad), 6)));
        ui->ikResultsTableWidget->setItem(
            row, 9, new QTableWidgetItem(Robot3DVisualizer::postureLabel(config_.posture, "shoulder", solution.posture.shoulder)));
        ui->ikResultsTableWidget->setItem(
            row, 10, new QTableWidgetItem(Robot3DVisualizer::postureLabel(config_.posture, "elbow", solution.posture.elbow)));
        ui->ikResultsTableWidget->setItem(
            row, 11, new QTableWidgetItem(Robot3DVisualizer::postureLabel(config_.posture, "wrist", solution.posture.wrist)));

        auto* totalCostItem = new QTableWidgetItem(formatNumber(solution.score.totalCost, 6));
        totalCostItem->setToolTip(
            QStringLiteral("seed=%1\nmotion=%2\nlimit=%3\nposture=%4")
                .arg(formatNumber(solution.score.seedDistanceCost, 6))
                .arg(formatNumber(solution.score.motionContinuityCost, 6))
                .arg(formatNumber(solution.score.jointLimitMarginCost, 6))
                .arg(formatNumber(solution.score.postureMismatchCost, 6)));
        ui->ikResultsTableWidget->setItem(row, 12, totalCostItem);
    }

    if (!result.solutions.empty()) {
        ui->ikResultsTableWidget->selectRow(0);
    }

    updateActionState();
}

void MainWindow::applySelectedIkSolution()
{
    const int row = ui->ikResultsTableWidget->currentRow();
    if (row < 0 || row >= static_cast<int>(lastIkSolutions_.size())) {
        updateIkStatus(QStringLiteral("Select an IK solution before applying it."));
        return;
    }

    const std::vector<double> degrees = lastIkSolutions_[static_cast<std::size_t>(row)].joints.toDegrees();
    std::array<double, 6> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
        values[i] = degrees[i];
    }
    setJointDegrees(values);
    // ui->controlTabWidget->setCurrentWidget(ui->fkTab);
    updateIkStatus(QStringLiteral("Applied IK solution #%1 to the joint controls.").arg(row + 1));
}

void MainWindow::setJointDegrees(const std::array<double, 6>& degrees)
{
    const std::array<QDoubleSpinBox*, 6> spins = jointSpinBoxes();
    for (std::size_t i = 0; i < spins.size(); ++i) {
        const QSignalBlocker blocker(spins[i]);
        spins[i]->setValue(degrees[i]);
    }
    applyJointStateToSceneAndReadouts();
}

JointVector MainWindow::currentJointVector() const
{
    const std::array<QDoubleSpinBox*, 6> spins = jointSpinBoxes();
    return JointVector::fromDegrees({
        spins[0]->value(),
        spins[1]->value(),
        spins[2]->value(),
        spins[3]->value(),
        spins[4]->value(),
        spins[5]->value(),
    });
}

std::array<QDoubleSpinBox*, 6> MainWindow::jointSpinBoxes() const
{
    return {{
        ui->joint1SpinBox,
        ui->joint2SpinBox,
        ui->joint3SpinBox,
        ui->joint4SpinBox,
        ui->joint5SpinBox,
        ui->joint6SpinBox,
    }};
}

std::array<QDoubleSpinBox*, 6> MainWindow::targetSpinBoxes() const
{
    return {{
        ui->targetXSpinBox,
        ui->targetYSpinBox,
        ui->targetZSpinBox,
        ui->targetRzSpinBox,
        ui->targetRySpinBox,
        ui->targetRxSpinBox,
    }};
}

std::array<QCheckBox*, 8> MainWindow::partVisibleCheckBoxes() const
{
    return {{
        ui->baseVisibleCheckBox,
        ui->j1VisibleCheckBox,
        ui->j2VisibleCheckBox,
        ui->j3VisibleCheckBox,
        ui->j4VisibleCheckBox,
        ui->j5VisibleCheckBox,
        ui->j6VisibleCheckBox,
        ui->toolVisibleCheckBox,
    }};
}

std::array<QCheckBox*, 8> MainWindow::partOriginCheckBoxes() const
{
    return {{
        ui->baseOriginCheckBox,
        ui->j1OriginCheckBox,
        ui->j2OriginCheckBox,
        ui->j3OriginCheckBox,
        ui->j4OriginCheckBox,
        ui->j5OriginCheckBox,
        ui->j6OriginCheckBox,
        ui->toolOriginCheckBox,
    }};
}

std::array<QCheckBox*, 8> MainWindow::partAxisCheckBoxes() const
{
    return {{
        ui->baseAxisCheckBox,
        ui->j1AxisCheckBox,
        ui->j2AxisCheckBox,
        ui->j3AxisCheckBox,
        ui->j4AxisCheckBox,
        ui->j5AxisCheckBox,
        ui->j6AxisCheckBox,
        ui->toolAxisCheckBox,
    }};
}

Result<Tool> MainWindow::selectedTool() const
{
    const QString toolId = ui->toolComboBox->currentData().toString();
    return toolId.isEmpty() ? toolRegistry_.getDefault() : toolRegistry_.get(ToolId{toolId.toStdString()});
}

Result<Pose> MainWindow::selectedReferenceInBase(const FkChain& chain) const
{
    const QString frameId = ui->referenceFrameComboBox->currentData().toString();
    if (frameId.isEmpty() || frameId == QStringLiteral("base")) {
        return Result<Pose>::success(Pose::identity());
    }

    const Result<UserFrame> frame = frameRegistry_.get(FrameId{frameId.toStdString()});
    if (!frame.ok()) {
        return Result<Pose>::failure(frame.status, frame.message);
    }
    return ForwardKinematics::userFrameInBase(chain, frame.value);
}

Result<std::optional<ArmPosture>> MainWindow::requestedPosture() const
{
    if (!postureResolver_) {
        return Result<std::optional<ArmPosture>>::success(std::nullopt);
    }

    std::map<std::string, std::string> labels;
    const auto capture = [&](QComboBox* comboBox, const std::string& axis) {
        const QString value = comboBox->currentData().toString();
        if (!value.isEmpty()) {
            labels[axis] = value.toStdString();
        }
    };

    capture(ui->shoulderRequestComboBox, "shoulder");
    capture(ui->elbowRequestComboBox, "elbow");
    capture(ui->wristRequestComboBox, "wrist");

    if (labels.empty()) {
        return Result<std::optional<ArmPosture>>::success(std::nullopt);
    }

    const Result<ArmPosture> posture = postureResolver_->fromLabels(config_.posture, labels);
    if (!posture.ok()) {
        return Result<std::optional<ArmPosture>>::failure(posture.status, posture.message);
    }
    return Result<std::optional<ArmPosture>>::success(posture.value);
}
