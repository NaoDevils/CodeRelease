/**
 * \file KalmanRobotMapProviderParameters.h
 * \author Heiner Walter <dino.menges@tu-dortmund.de>
 * 
 * This file contains parameters for the KalmanMultiRobotMapProvider.
 * Parameters are written to file "KalmanMultiRobotMapProvider.cfg".
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"


/**
 * \class KalmanRobotMapProviderParameters
 * 
 * This class contains all parameters which are required to generate the
 * remote robot map using a multiple kalman filter model.
 */
STREAMABLE(KalmanMultiRobotMapProviderParameters,
  /**
   * \class ValidityParameters
   * 
   * This class contains validity related parameters of the multiple kalman model
   * which differ between local and remote ball model.
   * 
   * \see BallModelProviderParameters::local, BallModelProviderParameters::remote
   */
  STREAMABLE(ValidityParameters,
    /**
     * Computes the minimum validity for a valid hypothesis from the parameters 
     * \c Validity_minPerceptsPerSecond and \c Validity_maxPerceptsPerSecond.
     * \return The minimum validity.
     */
    float minValidity() const
    { return static_cast<float>(validity_minPerceptsPerSecond) / static_cast<float>(validity_maxPerceptsPerSecond); }
  
    /**
     * Computes the validity required for a good hypothesis from the parameters
     * \c Validity_goodPerceptsPerSecond and \c Validity_maxPerceptsPerSecond.
     * \return The good validity threshold.
     */
    float goodValidity() const
    { return static_cast<float>(validity_goodPerceptsPerSecond) / static_cast<float>(validity_maxPerceptsPerSecond); }
    ,

    /// Use this validity for new ball hypotheses.
    (float) hypotheses_initialValidityForNewHypotheses,
  
    /// Defines the number of percepts/s which leads to a validity of 1.
    /// No percept leads to a validity of 0.
    (float) validity_maxPerceptsPerSecond,
    /// Defines the number of percepts/s which is required for a valid hypothesis.
    (float) validity_minPerceptsPerSecond,
    /// Defines the number of percepts/s which is required for a good validity.
	(float) validity_goodPerceptsPerSecond,
    /// The percepts/s from the last second is merged with the previous validity
    /// using a weighted mean. The last seconds percepts/s has weight 1 and this
    /// parameter defines the weight of the previous validity.
    (float) validity_weightOfPreviousValidity,
    /// Same as \c Validity_weightOfPreviousValidity for hypotheses with at least
    /// \c Validity_goodPerceptsPerSecond.
    (float) validity_weightOfPreviousValidity_goodHypotheses,
    /// If the validity of a robot hypothesis is less than this threshold (and there
    /// is a better one), the hypothesis will be removed.
    (float) cleanUpHypotheses_belowValidity
  );

  STREAMABLE(LocalKalmanMapParameters,,
    // Distance up to which the percepts are merged using a distance-based
    // approach. If the percepts are further away, an angle-based approach is used.
    (float) maxDistanceForDistanceBasedMerging,
    /// Max distance to merge hypotheses
    (float) maxDistanceToMerge,
    /// If a ball percept has at least this distance to all existing ball
    /// hypotheses, a new hypothesis is created with position from ball percept
    /// (distance in mm).
    (Vector2a) maxAngleDiffToMerge,
    /// Max difference (percentage) for robot types to merge
    (float) maxRobotTypeDifferenceToMerge
  );
  
  STREAMABLE(RemoteKalmanMapParameters,,
    // Distance up to which the percepts are merged using a distance-based
    // approach. If the percepts are further away, an angle-based approach is used.
    (float) maxDistanceForDistanceBasedMerging,
    /// Max distance to merge hypotheses
    (float) maxDistanceToMerge,
    /// If a ball percept has at least this distance to all existing ball
    /// hypotheses, a new hypothesis is created with position from ball percept
    /// (distance in mm).
    (Vector2a) maxAngleDiffToMerge,
    /// Max difference (percentage) for robot types to merge
    (float) maxRobotTypeDifferenceToMerge,
    /// The influence of a teammate position
    (float) teammatePositionInfluence,
    /// The influence of a teammate's robot model
    (float) teammateModelInfluence,
    /// The influence of a teammate's percept
    (float) teammatePerceptInfluence
  );
  
  STREAMABLE(MergedMapParameters,,
    // Distance up to which the percepts are merged using a distance-based
    // approach. If the percepts are further away, an angle-based approach is used.
    (float) maxDistanceForDistanceBasedMerging,
    /// Max distance to merge hypotheses
    (float) maxDistanceToMerge,
    /// If a ball percept has at least this distance to all existing ball
    /// hypotheses, a new hypothesis is created with position from ball percept
    /// (distance in mm).
    (Vector2a) maxAngleDiffToMerge,
    /// Max difference (percentage) for robot types to merge
    (float) maxRobotTypeDifferenceToMerge,
    /// The influence of a local robot percept
    (float) localPerceptInfluence,
    /// The influence of a teammate position
    (float) teammatePositionInfluence,
    /// The influence of a teammate's robot model
    (float) teammateModelInfluence,
    /// The influence of a teammate's percept
    (float) teammatePerceptInfluence
  );
  ,
  
  /// Validity related parameters of the multi kalman model for local percepts.
  (ValidityParameters) localPercept,
  /// Validity related parameters of the multi kalman model for remote models.
  (ValidityParameters) remoteModel,
  
  /// Parameter to handle local map
  (LocalKalmanMapParameters) localMapParameters,
  /// Parameter to handle remote map
  (RemoteKalmanMapParameters) remoteMapParameters,
  /// Parameter to handle combined map
  (MergedMapParameters) mergedMapParameters,
  
  /// Slow down the velocity of the fitered robot hypotheses when unseen.
  /// The friction value must be <= 0 (0 if no friction should be applied).
  (float) friction,
  
  /// If \c true hypotheses outside the field (with \c CleanUpHypotheses_fieldBorderThreshold)
  /// are removed (also the best one).
  /// Warning: This could be dangerous if the localization is too inaccurate!
  (bool) cleanUpHypotheses_outsideField,
  /// The field border (all legal robot positions are inside) is enlarged by this
  /// size (in mm) for clean up. Only hypotheses outside this threshold are removed.
  /// See \c CleanUpHypotheses_outsideField.
  (float) cleanUpHypotheses_fieldBorderThreshold
);
