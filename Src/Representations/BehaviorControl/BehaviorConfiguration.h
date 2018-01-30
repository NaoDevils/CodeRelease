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
#include "Modules/BehaviorControl/BehaviorControl2015/Behavior2015Parameters.h"

STREAMABLE(BehaviorConfiguration,
{ ,

  (float)(1000) gotoThreshMaxTime,
  (float)(80) gotoBaseThresh,
  (Angle)(10_deg) gotoBaseThreshRot,
  (float)(0.2f) isLocalizedMinValidity,
  (bool)(false) isLocalizedNoSymmetrie,
  (bool)(false) noWLAN,
  (float)(190.f) optDistanceToBallX,
  (float)(55.f) optDistanceToBallY,

  (float) targetDistRobotToTargetAngleFactor,
  (float) targetDistanceRobotRotFactor,
  (float) targetDistanceObstacleFactor,

  (Behavior2015Parameters) behavior2015Parameters,

});
