#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <RobotKinematics/Kinematics/ForwardKinematics.h>
#include <RobotKinematics/Kinematics/SerialRobotKinematics.h>
#include <RobotKinematics/Model/FrameRegistry.h>
#include <RobotKinematics/Model/ToolRegistry.h>
#include <RobotKinematics/Posture/PostureResolver.h>

#include <vtkSmartPointer.h>

#include <array>
#include <memory>
#include <vector>

class QVTKOpenGLNativeWidget;
class QCheckBox;
class QDoubleSpinBox;
class QTableWidget;
class vtkGenericOpenGLRenderWindow;
class vtkActor;
class vtkAxesActor;
class vtkLineSource;
class vtkOrientationMarkerWidget;
class vtkRenderer;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    struct VisualPartState {
        QString key;
        QString displayName;
        std::string linkId;
        RobotKinematics::Pose homeLinkInBase = RobotKinematics::Pose::identity();
        RobotKinematics::Pose homeVisualCorrection = RobotKinematics::Pose::identity();
        int jointAxisIndex = -1;
        bool isLoaded = false;
        vtkSmartPointer<vtkActor> actor;
        vtkSmartPointer<vtkAxesActor> originActor;
        vtkSmartPointer<vtkLineSource> axisSource;
        vtkSmartPointer<vtkActor> axisActor;
    };

    void setupVtkViewport();
    void setupModelState();
    void setupUiState();
    void connectSignals();
    void loadRobotVisuals();
    void applyJointStateToSceneAndReadouts();
    void updateSceneFromChain(const RobotKinematics::FkChain& chain);
    void updatePoseReadouts(const RobotKinematics::FkChain& chain);
    void updateJointStatus(const RobotKinematics::JointVector& joints);
    void updateCurrentPosture(const RobotKinematics::JointVector& joints);
    void updateIkStatus(const QString& message);
    void updateActionState();
    void populateCombos();
    void populateJointControls();
    void populatePostureControls();
    void populateSampleButtons();
    void populateDebugControls();
    void applyDebugVisualState();
    void resetTargetToCurrentTcp();
    void solveInverseKinematics(bool solveAll);
    void populateIkResults(const RobotKinematics::IKResult& result);
    void applySelectedIkSolution();
    void setJointDegrees(const std::array<double, 6>& degrees);
    RobotKinematics::JointVector currentJointVector() const;
    std::array<QDoubleSpinBox*, 6> jointSpinBoxes() const;
    std::array<QDoubleSpinBox*, 6> targetSpinBoxes() const;
    std::array<QCheckBox*, 8> partVisibleCheckBoxes() const;
    std::array<QCheckBox*, 8> partOriginCheckBoxes() const;
    std::array<QCheckBox*, 8> partAxisCheckBoxes() const;
    RobotKinematics::Result<RobotKinematics::Tool> selectedTool() const;
    RobotKinematics::Result<RobotKinematics::Pose> selectedReferenceInBase(
        const RobotKinematics::FkChain& chain) const;
    RobotKinematics::Result<std::optional<RobotKinematics::ArmPosture>> requestedPosture() const;

    Ui::MainWindow *ui;
    QVTKOpenGLNativeWidget* vtkWidget_ = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow_;
    vtkSmartPointer<vtkOrientationMarkerWidget> orientationMarker_;
    vtkSmartPointer<vtkRenderer> renderer_;
    RobotKinematics::SerialRobotConfig config_;
    RobotKinematics::SerialRobotKinematics robot_;
    RobotKinematics::FrameRegistry frameRegistry_;
    RobotKinematics::ToolRegistry toolRegistry_;
    std::unique_ptr<RobotKinematics::PostureResolver> postureResolver_;
    std::vector<VisualPartState> visualParts_;
    std::vector<RobotKinematics::IKSolution> lastIkSolutions_;
    QString assetsDirectory_;
};
#endif // MAINWINDOW_H
