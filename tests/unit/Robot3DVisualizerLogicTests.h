#pragma once

#include <QObject>

class Robot3DVisualizerLogicTests : public QObject
{
    Q_OBJECT

private slots:
    void convertsBetweenNachiPendantOrderAndPose();
    void extractsCanonicalZyxEulerAngles();
    void buildsMillimeterVisualDeltaMatrixFromHomePose();
    void appliesCenteringToolHomeVisualCorrection();
    void mapsPostureBranchesToConfiguredLabels();
    void formatsKinematicsStatusesForUi();
    void derivesSimplifiedMeshProfilePathBesideOriginal();
};
