#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/MotionControl/SpecialActionRequest.h"

/**
 * Contains parameters that are shared between several motion modules.
 */
STREAMABLE(MotionSettings,
{,
  (float)(262.0f) comHeight, /**< Height of the com above the ground when walking/standing/kicking. */
  ((SpecialActionRequest) SpecialActionID) standUpMotionFront,
  ((SpecialActionRequest) SpecialActionID) standUpMotionBack,
  (bool)(false) leaveWalkForDive, /**< If true, walk is stopped and aborted immediately for diving motion */
});
