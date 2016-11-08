/**
* @file Representations/MotionControl/ArmMovement.h
* This file declares a class that represents the output of ArmAnimator.
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Math/Pose2f.h"

/**
* @class ArmMovement
* A class that represents the output of the walking engine.
*/
STREAMABLE_WITH_BASE(ArmMovement, JointAngles,
{,
  (bool)(false) usearms,
  (bool)(false) armsInContactAvoidance,
});
