/**
 * \file KalmanRobotMapProviderParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 * 
 * This file contains parameters for the KalmanRobotMapProvider.
 * Parameters are written to file "KalmanRobotMapProvider.cfg".
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "../BallModelProvider/BallModelProviderParameters.h"

/**
 * \class KalmanRobotMapProviderParameters
 * 
 * This class contains all parameters which are required to generate the
 * remote robot map using a multiple kalman filter model.
 */
STREAMABLE(KalmanRobotMapProviderParameters,
{,
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
  (float) CleanUpHypotheses_fieldBorderThreshold,
});
