/**
 * \file TeamBallModelParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 * 
 * This file contains parameters for the TeamBallModelProvider which is a 
 * part of the module BallModelProvider.
 * Parameters are written to file "ballModelProvider.cfg".
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(TeamBallModelParameters,
{,
  /// This Validity (in range [0,1]) is at least required for the local 
  /// ball model to believe it is correct.
  (float)(0.35f) minValidityForLocalBallModel,

  /// This Validity (in range [0,1]) is at least required for the remote 
  /// ball model (only other teammates) to believe it is correct.
  /// The remote ball model is only used, if the local ball models 
  /// validity is below \c minValidityForLocalBallModel.
  (float)(0.5f) minValidityForRemoteBallModel,
});
