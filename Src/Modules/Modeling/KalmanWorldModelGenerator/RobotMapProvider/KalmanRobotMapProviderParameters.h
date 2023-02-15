/**
 * \file KalmanRobotMapProviderParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 * 
 * This file contains parameters for the KalmanRobotMapProvider.
 * Parameters are written to file "KalmanRobotMapProvider.cfg".
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"


/**
 * \class KalmanRobotMapProviderParameters
 * 
 * This class contains all parameters which are required to generate the
 * remote robot map using a multiple kalman filter model.
 */
STREAMABLE(KalmanRobotMapProviderParameters,
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
  /// Validity related parameters of teh multi kalman model.
  (ValidityParameters) validity,
  
  /// Slow down the velocity of the fitered robot hypotheses when unseen.
  /// The friction value must be <= 0 (0 if no friction should be applied).
  (float) friction,
  
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
  (float) CleanUpHypotheses_fieldBorderThreshold
);
