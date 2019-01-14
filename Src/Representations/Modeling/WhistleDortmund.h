#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WhistleDortmund,
{
  ENUM(DetectionState,
    { ,
        dontKnow,
        notDetected,
        isDetected,
    }),

  (DetectionState)(dontKnow) detectionState, /**< Was detected? */
  (bool)(false) detected, /**< Was detected? */
});

STREAMABLE(WhistleCausedPlay,
{ ,
  (bool)(false) whistleCausedPlay, /** majority decided on playing, communicate this! */
});