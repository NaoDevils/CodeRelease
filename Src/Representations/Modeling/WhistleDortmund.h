#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WhistleDortmund,
  ENUM(DetectionState,
        dontKnow,
        notDetected,
        isDetected
    ),

  (DetectionState)(dontKnow) detectionState, /**< Was detected? */
  (unsigned int)(0) lastDetectionTime,
  (float)(0.0) lastConfidence
);
