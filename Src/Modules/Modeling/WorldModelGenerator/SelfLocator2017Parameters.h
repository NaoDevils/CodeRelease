/**
*
* This file contains parameters for the SelfLocator2017.
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
//#include "Platform/GTAssert.h"

STREAMABLE(SelfLocator2017Parameters,
{
  STREAMABLE(ProcessUpdate,
  { ,
    // Determines whether to update the coVar matrix based on odometry data
    (bool)(false) odometryBasedVarianceUpdate,
    (float)(5.f) minPositionChageForCovarianceUpdate,
    (float)(0.01f) minRotationChageForCovarianceUpdate,
    (float)(50.f) positionVariance,
    (float)(0.001f) rotationVariance,

    (float)(0.0f) adjustRotationToBestFittingAngle,

    (float)(0.15f) positionConfidenceHysteresisForKeepingBestHypothesis,
  });

  STREAMABLE(SensorUpdate,
  { ,
    (float)(0.08f) verticalAngleVariance,
    (float)(0.2f) horizontalAngleVariance,
    (float)(0.008f) projectiveNormalVariance,
    (float)(1.2f) maxVerticalAngleFullObservationWeight, // [0..pi_2]

    (float)(0.5f) correlationFactorBetweenMeasurements, // [0..1]
    (float)(0.007f) influenceOfNewCenterCircleMeasurementOnPositionConfidence, // [0..1]
    (float)(0.004f) influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, // [0..1]
    (float)(0.003f) influenceOfNewGoalMeasurementOnPositionConfidence, // [0..1]
    (float)(0.005f) influenceOfNewLineMeasurementOnPositionConfidence, // [0..1]
    (float)(0.003f) influenceOfNewInfiniteLineMeasurementOnPositionConfidence, // [0..1]
    (float)(0.2f) maxInfluenceOnPositionConfidencePenaltyCrossOnly, // [0..1]
    (bool)(true) use1stLevelUpdate,
    (bool)(true) use2ndLevelUpdate,
    (bool)(true) use3rdLevelUpdate,

    (float)(300.f) minLineLengthOnField, // factor 0
    (float)(0.5f) worstAngleDifference, // factor 0
    (float)(0.3f) maxDistanceError, // (expected dist / real dist) error
    (float)(1.2f) lineLengthMatchFactorMax,
    (float)(0.75f) confidenceScaleFactorAfterFallDown,
  });

  STREAMABLE(SymmetryUpdate,
  { ,
    (bool)(true) updateWithRemoteModels,
    (float)(500.f) maxDistanceToClosestRemoteModel, // max distance between local and remote model for doing symmetry update (in mm)
    (float)(0.02f) influenceOfNewMeasurement, // [0..1]
    (float)(0.02f) influenceOfNewMeasurementByGoalie, // [0..1]
  });

  STREAMABLE(LocalizationStateUpdate,
  { ,
    (bool)(false) symmetryLostWhenFallDownInCenterCircle,
    (float)(200.f) unknownSymmetryRadiusAroundCenter,

    (float)(0.2f) positionLostWhenBestConfidenceBelowThisThreshold,
    (float)(0.5f) positionFoundAgainWhenBestConfidenceAboveThisThreshold,

    (float)(0.1f) symmetryLostWhenBestConfidenceBelowThisThreshold,
    (float)(0.8f) symmetryFoundAgainWhenBestConfidenceAboveThisThreshold,
  });

  STREAMABLE(Spawning,
  {
    // This is used as a flag in SelfLocator2017::addNewHypotheses
    ENUM(LandmarkBasedHypothesesSpawn,
    {,
      off,
      spawnIfPositionLost,
      spawnIfPositionTracking,
      spawnIfLostOrTracking,
    }),

    // Adding new Hypotheses
    (bool)(0.1f) useOdometryForSpawning,
    (LandmarkBasedHypothesesSpawn)(spawnIfPositionLost) landmarkBasedHypothesesSpawn,

    (float) spawnWhilePositionTrackingWhenBestConfidenceBelowThisThreshold, // [0..1]
    // Nearest pose (symmetric vs. own side) is determined by best hypothesis.
    // This threshold allows other hypothesis with a confidence lower than the best hypothesis
    // also to be taken into account for check of closest pose when within this interval (best confidence - value)
    (float)(0.1f) confidenceIntervalForCheckOfHypothesis, // [0..1]

    (unsigned char)(8) noAdditionalHypothesisAfterFallDown,

    (float)(0.3f) positionConfidenceWhenPositionedManually,
    (float)(0.4f) positionConfidenceWhenPositionedManuallyForGoalKeeper,
    (float)(0.3f) positionConfidenceWhenPositionLost,

    (float)(1.0f) symmetryConfidenceWhenPositionedManually,

    (float)(0.8f) goalBaseConfidence,
    (float)(0.8f) centerCircleBaseConfidence,
    (float)(0.8f) penaltyCrossBaseConfidence,
    (float)(0.25f) lineBasedPositionConfidenceWhenPositionTracking,
  });

  STREAMABLE(Pruning,
  { ,
    (float)(0.5f) likelihoodTresholdForMerging,
    (unsigned char)(12) maxNumberOfHypotheses,
  });

  STREAMABLE(Matching,
  { ,
    (float)(250000.f) positionVarianceForLines,
    (float)(0.25f) orientationVarianceForLines,
    (float)(0.1f) likelihoodTresholdForLines,

    (float)(0.3f) maxAllowedVerticalAngleDifferenceForPoints,
    (float)(0.3f) maxAllowedHorizontalAngleDifferenceForPoints,
  });

  STREAMABLE(SensorReset,
  { ,
    (float)(100.f) maxDistanceForLocalResetting,
    (float)(0.8f) maxAngleForLocalResetting,
  });

  STREAMABLE(Debugging,
  { ,
    (float)(5000.f) durationHighlightAddedHypothesis,
    (bool)(false) displayWarnings,
  });
  ,

  (ProcessUpdate) processUpdate,
  (SensorUpdate) sensorUpdate,
  (SymmetryUpdate) symmetryUpdate,
  (LocalizationStateUpdate) localizationStateUpdate,
  (Spawning) spawning,
  (Pruning) pruning,
  (Matching) matching,
  (SensorReset) sensorReset,
  (Debugging) debugging,
});

STREAMABLE(PositionsByRules,
{ ,
  (std::vector<Vector2f>) penaltyPositions,
  (std::vector<Vector2f>) fieldPlayerPositionsOwnKickoff,
  (std::vector<Vector2f>) fieldPlayerPositionsOppKickoff,
  (Vector2f) goaliePosition,
  (Vector2f) penaltyShootOutPosition,
});

