/**
 * \file BallModelProviderParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 * 
 * This file contains parameters for the BallModelProvider.
 * Parameters are written to file "ballModelProvider.cfg".
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"


/**
 * \class BallModelProviderParameters
 * 
 * This class contains all parameters which are required to generate the
 * (local and remote) ball model. All parameters which differ between local 
 * and remote ball model are stored in an instance each of the class 
 * \c ValidityParameters.
 * 
 * \see ValidityParameters
 */
STREAMABLE(BallModelProviderParameters,
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
    { return static_cast<float>(Validity_minPerceptsPerSecond) / static_cast<float>(Validity_maxPerceptsPerSecond); }
  
    /**
     * Computes the validity required for a good hypothesis from the parameters
     * \c Validity_goodPerceptsPerSecond and \c Validity_maxPerceptsPerSecond.
     * \return The good validity threshold.
     */
    float goodValidity() const
    { return static_cast<float>(Validity_goodPerceptsPerSecond) / static_cast<float>(Validity_maxPerceptsPerSecond); }
    ,
  
    /// If a ball percept has at least this distance to all existing ball 
    /// hypotheses, a new hypothesis is created with position from ball percept 
    /// (distance in mm).
    (float) Hypotheses_minDistanceForNewHypothesis,
    /// If a ball percept has at least this distance to all existing ball 
    /// hypotheses, a new hypothesis is created with position from ball percept 
    /// (distance in mm).
    (Vector2a) Hypotheses_mergeAngleDiff,
    /// Use this validity for new ball hypotheses.
    (float) Hypotheses_initialValidityForNewHypotheses,
  
    /// Only hypotheses with at least this validity can become the best hypothesis
    /// of a \c MultipleKalmanModel. If all hypotheses are below this threshold
    /// the last best hypothesis is retained.
    (float) Hypotheses_minValidityForChangingBestHypothesis,
  
    /// Defines the number of percepts/s which leads to a validity of 1.
    /// No percept leads to a validity of 0.
    (float) Validity_maxPerceptsPerSecond,
    /// Defines the number of percepts/s which is required for a valid hypothesis.
    (float) Validity_minPerceptsPerSecond,
    /// Defines the number of percepts/s which is required for a good validity.
    (float) Validity_goodPerceptsPerSecond,
    /// The percepts/s from the last second is merged with the previous validity
    /// using a weighted mean. The last seconds percepts/s has weight 1 and this
    /// parameter defines the weight of the previous validity.
    (float) Validity_weightOfPreviousValidity,
    /// Same as \c Validity_weightOfPreviousValidity for hypotheses with at least
    /// \c Validity_goodPerceptsPerSecond.
    (float) Validity_weightOfPreviousValidity_goodHypotheses
  );
  ,
  /// Specific parameters for the local ball model (\c BallModel) which differs
  /// from the \c remote parameters.
  (ValidityParameters) local,
  /// Specific parameters for the remote ball model which differs from the
  /// \c local parameters.
  (ValidityParameters) remote,
  

  /// If two hypotheses are closer than Hypotheses_minDistanceForNewHypothesis
  /// and the angle between their velocities is less than this parameter, one of 
  /// them will be deleted during clean up (angle in radian).
  (float) CleanUpHypotheses_minAngleForKeepingHypothesis,
  /// If the validity of a ball hypothesis is less than this threshold (and there 
  /// is a better one), the hypothesis will be removed.
  (float) CleanUpHypotheses_belowValidity,
  
  /// If \c true hypotheses outside the field (with \c CleanUpHypotheses_fieldBorderThreshold)
  /// are removed (also the best one).
  /// Warning: This could be dangerous if the localization is too inaccurate!
  (bool) CleanUpHypotheses_outsideField,
  /// The field border (all legal ball positions are inside) is enlarged by this
  /// size (in mm) for clean up. Only hypotheses outside this threshold are removed.
  /// See \c CleanUpHypotheses_outsideField.
  (float) CleanUpHypotheses_fieldBorderThreshold,

  /// Only hypotheses which had at least this amount of sensor updates are
  /// allowed to become the \c bestHypothesis.
  (int) Hypotheses_minNumberOfSensorUpdatesForBestHypothesis,
  (int) Hypotheses_minNumberOfSensorUpdatesForBestHypothesisRemote,
  /// If the best hypothesis changes, the last ones validity is decreased by this value.
  (float) Hypotheses_decreaseValidityOnChangingBestHypothesis,
  
  /// If \c true add a hypothesis at the kick off point when game state changes
  /// from \c SET to \c PLAYING.
  (bool) State_SetToPlaying_addKickOffHypothesis,

  /// If \c true add a hypothesis at the defined set play points when set play state changes
  /// to either \c SET_PLAY_CORNER_KICK or to \c SET_PLAY_GOAL_KICK
  (bool) State_SetPlay_addSetPlayHypothesis,

  /// Maximum time that can pass since ball model update to add fake hypthesis on SET_PLAY
  (int) State_SetPlay_maxTimeSinceBallSeen,
  
  /// If \c true all ball hypotheses are removed after a penalty.
  (bool) State_Penalty_removeAllHypotheses
);
