#pragma once

#include <string.h>

#include "Representations/MotionControl/KickRequest.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Angle.h"

STREAMABLE(PenaltyShootout2017Parameters,
  STREAMABLE(Striker,,
    (float)(0.5f) probabilityToKickLeft,
    (float)(210.f) optDistanceToBallX, /**< theFieldDimensions.ballRadius is added on load! */
    (float)(55.f) optDistanceToBallY, /**< theFieldDimensions.ballRadius is added on load! */
    (Angle)(18_deg) rotation, /**< theFieldDimensions.ballRadius is added on load! */
    ((KickRequest) KickMotionID)(mostPowerfulKick) kickIdKickLeft,
    (bool)(false) mirrorKickLeft,
    ((KickRequest) KickMotionID)(mostPowerfulKick) kickIdKickRight,
    (bool)(false) mirrorKickRight,
    (bool)(false) correctPosition,
    (int)(45) durationInSecs,
    (int)(10) secsToKick,
    (float)(0.5f) positionConfidenceBeforeStarting,
    (int)(15) maxSecsToLocalize, 
    (float)(10.f) maxXDeviationFrontForKick,
    (float)(20.f) maxXDeviationBackForKick,
    (float)(10.f) maxYDeviationForKick,
    (Angle)(3_deg) maxAngleDeviationForKick
  );

  STREAMABLE(Goalie,,
    // DEPRECATED: used for old keeper (see PenaltyKeeper2017.h)
    // currently PSGoalTrigger.h/cpp and PenaltyKeeper.h is used.
    (float)(50.f) xOffsetFromPenaltyArea,
    (float)(650.f) distToBallForTrigger,
    (float)(1100.f) distToBallForTriggerDive,
    (int)(5000) timeLeftPSForTrigger,
    (float)(-50.f) velOfBallForTrigger,
    (float)(150.f) stepForwardSpeed,
    (float)(3500.f) stepForwardDuration,
    (bool)(false) useGoalieDiveBehavior,
    (int)(500) blockArea,
    (bool)(false) useSpeculativeMode,
    (int)(12000) checkRobotAtTimeSpeculative
  );
  ,

  (Striker) striker,
  (Goalie) goalie
);
