#pragma once

#include <QObject>

class NachiMZ07FTests : public QObject
{
    Q_OBJECT

private slots:
    void fallbackPresetIsValidAndHasRequiredMetadata();
    void jsonPresetMatchesCppFallbackForSolverFacingFields();
    void forwardKinematicsMatchesTeachPendantPoses();
    void postureClassificationUsesNachiLabels();
    void presetRunsFkAndSeededIkRoundTrip();
    void primitiveCollisionProfileIsValidAndClearAtReferencePoses();
    void meshCollisionProfileLoadsMeterStlAssets();
    void meshToLinkReproducesStepAssemblyPlacement();
    void meshBackendReportsNoSelfCollisionAtReferencePosesWhenCompiled();
    void meshBackendDetectsFoldedSelfCollisionWhenCompiled();
};
