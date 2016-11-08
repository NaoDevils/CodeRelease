/**
*
* This file contains parameters for the SelfLocator2016.
*
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
//#include "Platform/GTAssert.h"


STREAMABLE(SelfLocator2016Parameters,
{
  ENUM(LandmarkBasedHypothesesSpawn,
  {,
    off,
    spawnIfPositionLost,
    spawnIfPositionTracking,
    spawnIfLostOrTracking,
  }),

  (float) sensorUpdate_verticalAngleVariance,
  (float) sensorUpdate_horizontalAngleVariance,
  (float) sensorUpdate_projectiveNormalVariance,
  (float) sensorUpdate_maxVerticalAngleFullObservationWeight, // [0..pi_2]

  (bool) processUpdate_odometryBasedVarianceUpdate, // Determines whether to update the coVar matrix based on odometry data
  (float) processUpdate_minPositionChageForCovarianceUpdate,
  (float) processUpdate_minRotationChageForCovarianceUpdate,
  (float) processUpdate_positionVariance,
  (float) processUpdate_rotationVariance,

  (float) sensorUpdate_correlationFactorBetweenMeasurements, // [0..1]
  (float) sensorUpdate_influenceOfNewCenterCircleMeasurementOnPositionConfidence, // [0..1]
  (float) sensorUpdate_influenceOfNewPenaltyCrossMeasurementOnPositionConfidence, // [0..1]
  (float) sensorUpdate_influenceOfNewGoalMeasurementOnPositionConfidence, // [0..1]
  (float) sensorUpdate_influenceOfNewLineMeasurementOnPositionConfidence, // [0..1]
  (float) sensorUpdate_influenceOfNewInfiniteLineMeasurementOnPositionConfidence, // [0..1]
  (float) sensorUpdate_maxInfluenceOnPositionConfidencePenaltyCrossOnly, // [0..1]
  (bool) sensorUpdate_use1stLevelUpdate,
  (bool) sensorUpdate_use2ndLevelUpdate,
  (bool) sensorUpdate_use3rdLevelUpdate,

  //(float) sensorUpdate_optLineLengthOnField, // factor 1
  (float) sensorUpdate_minLineLengthOnField, // factor 0
  (float) sensorUpdate_worstAngleDifference, // factor 0
  (float) sensorUpdate_maxDistanceError, // (expected dist / real dist) error
  (float) sensorUpdate_lineLengthMatchFactorMax,

  (float) sensorUpdate_adjustRotationToBestFittingAngle,

  (float) lineMatching_positionVariance,
  (float) lineMatching_orientationVariance,
  (float) lineMatching_likelihoodTreshold,

  (float) pointMatching_maxAllowedVerticalAngleDifference,
  (float) pointMatching_maxAllowedHorizontalAngleDifference,

  (float) pruning_likelihoodTresholdForMerging,
  (float) pruning_maxNumberOfHypotheses,

  (float) sensorResetting_maxDistanceForLocalResetting,
  (float) sensorResetting_maxAngleForLocalResetting,

  (bool) symmetryUpdate_updateWithRemoteModels,
  (float) symmetryUpdate_maxDistanceToClosestRemoteModel, // max distance between local and remote model for doing symmetry update (in mm)
  (float) symmetryUpdate_influenceOfNewMeasurement, // [0..1]
  (float) symmetryUpdate_influenceOfNewMeasurementByGoalie, // [0..1]

  (float) localizationState_unknownSymmetryRadiusAroundCenter,
  (float) localizationState_positionLostWhenBestConfidenceBelowThisThreshold,
  (float) localizationState_symmetryLostWhenBestConfidenceBelowThisThreshold,
  (float) localizationState_symmetryFoundAgainWhenBestConfidenceAboveThisThreshold,
  (bool) localizationState_symmetryLostWhenFallDownInCenterCircle,
  (float) positionConfidence_scaleFactorAfterFallDown,

  // Adding new Hypotheses
  (bool) useOdometryForSpawning,
  (LandmarkBasedHypothesesSpawn)(spawnIfPositionLost) landmarkBasedHypothesesSpawn,

  (float) spawning_spawnWhilePositionTrackingWhenBestConfidenceBelowThisThreshold, // [0..1]

  (float) landmarkBasedHypothesesSpawn_goalBaseConfidence,
  (float) landmarkBasedHypothesesSpawn_centerCircleBaseConfidence,
  (float) landmarkBasedHypothesesSpawn_penaltyCrossBaseConfidence,

  (float) spawning_positionConfidenceWhenPositionedManually,
  (float) spawning_positionConfidenceWhenPositionedManuallyForGoalKeeper,
  (float) spawning_lineBasedPositionConfidenceWhenPositionTracking,
  (float) spawning_positionConfidenceWhenPositionLost,

  (float) symmetryConfidence_whenPositionedManually,
          
  (float) debug_durationHighlightAddedHypothesis,
  
  (bool) displayWarnings,

});

STREAMABLE(PositionsByRules,
{ ,
  (std::vector<Vector2f>) penaltyPositions,
  (std::vector<Vector2f>) fieldPlayerPositionsOwnKickoff,
  (std::vector<Vector2f>) fieldPlayerPositionsOppKickoff,
  (Vector2f) goaliePosition,
  (Vector2f) penaltyShootOutPosition,
});

