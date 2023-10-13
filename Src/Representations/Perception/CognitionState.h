#pragma once

#include "Tools/Streams/AutoStreamable.h"

#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/WalkRequest.h"

STREAMABLE(PenaltyStatus,,
  (int)(0) penaltyScore
);

STREAMABLE(LocaStatus,,
  (float)(0) locaScore
);

STREAMABLE(BallStatus,,
  (int)(0) ballDistance,
  (int)(0) ballLostAfterKick,
  (bool)(false) isKickEngine,
  (double)(0.0) footSpeed,
  (int)(KickRequest::KickMotionID::none) kickEngineID,
  (int)(WalkRequest::StepRequest::none) walkKickID,
  (float)(0) estimatedKickDistance,
  (double)(0) swingDistance,
  (double)(0) swingTime,
  (bool)(false) timeout,
  (double)(0) simulatedKickDistance,
  (double)(0.24) rollResistance,
  (Angle)(0_deg) targetAngle,
  (Vector2f)(0.f,0.f) ballPositionPrediction
);

STREAMABLE(ImageStatus,,
    (int)(0) lowerCameraBrightness,
    (int)(0) upperCameraBrightness
  );

STREAMABLE(BatteryStatus,,
  (bool)(false) batteryBroken
);

STREAMABLE(CognitionState,
  ENUM(CognitionStateError,
    locaProblem,
    penaltyProblem,
    ballLostProblem,
    cameraDeviceLost,
    lowerCameraBrightnessLow,
    lowerCameraBrightnessBright,
    upperCameraBrightnessLow,
    upperCameraBrightnessBright
  ),

  (PenaltyStatus) penaltyStatus,
  (LocaStatus) locaStatus,
  (BallStatus) ballStatus,
  (ImageStatus) imageStatus,
  (BatteryStatus) batteryStatus,
  (std::vector<CognitionStateError>) cognitionProblems
);
