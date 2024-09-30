/**
 * @file BehaviorConfiguration.h
 * Declaration of class BehaviorConfiguration
 *
 * @author Ingmar Schwarz
 */

#pragma once

#include <string>
#include <vector>
#include "Tools/Streams/AutoStreamable.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"

STREAMABLE(BehaviorConfiguration,,

  (float)(1000) gotoThreshMaxTime,
  (float)(80) gotoBaseThresh,
  (Angle)(10_deg) gotoBaseThreshRot,
  (float)(0.2f) isLocalizedMinValidity,
  (bool)(false) isLocalizedNoSymmetrie,
  (float)(190.f) optDistanceToBallX,
  (float)(55.f) optDistanceToBallY,
  (float)(190.f) optLongkickDistanceToBallX,
  (float)(55.f) optLongkickDistanceToBallY,
  (float)(210.f) optKickoffDistanceToBallX,
  (float)(43.f) optKickoffDistanceToBallY,

  (float) targetDistRobotToTargetAngleFactor,
  (float) targetDistanceRobotRotFactor,
  (float) targetDistanceObstacleFactor,

  (BehaviorParameters) behaviorParameters

);
