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
  (unsigned int)(0) lastDetectionTime,
});

STREAMABLE(DistributedWhistleDortmund,
{
  DistributedWhistleDortmund() = default;
  DistributedWhistleDortmund(const WhistleDortmund& whistle, bool causedPlay)
  {
    this->whistle = whistle;
    whistleCausedPlay = causedPlay;
  }
  ,

  (WhistleDortmund) whistle,
  (bool)(false) whistleCausedPlay, /** majority decided on playing, communicate this! */
});
