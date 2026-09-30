#pragma once

#include <QObject>

class NachiMZ04DTests : public QObject
{
    Q_OBJECT

private slots:
    void fallbackPresetIsValidAndHasRequiredMetadata();
    void jsonPresetMatchesCppFallbackForSolverFacingFields();
    void jointLimitsFollowTeachPendantNotManual();
    void forwardKinematicsMatchesTeachPendantPoses();
    void forwardKinematicsWithToolMatchesMeasuredPoses();
    void postureClassificationMatchesNachiManualRules();
    void presetRunsFkAndSeededIkRoundTrip();
};
