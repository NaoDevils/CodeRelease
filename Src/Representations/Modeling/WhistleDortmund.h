#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WhistleDortmund,
  ENUM(DetectionState,
        dontKnow,
        notDetected,
        isDetected
  )

  ENUM(Microphone,
        rearLeft,
        rearRight,
        frontLeft,
        frontRight
  ),

  (DetectionState)(dontKnow) detectionState, /**< Was detected? */
  (Microphone)(rearLeft) currentMic,
  (unsigned int)(0) lastDetectionTime,
  (int)(0) detectedWhistleFrequency,
  (int)(0) minFrequency,
  (int)(0) maxFrequency,
  (float)(0.f) lastConfidence
);

STREAMABLE(WhistleDortmundCompressed,
  // Increase version number whenever something changes!
  static constexpr unsigned char version = 0;

  WhistleDortmundCompressed() = default;
  explicit WhistleDortmundCompressed(const WhistleDortmund& whistleDortmund)
    : lastDetectionTime(whistleDortmund.detectionState == WhistleDortmund::DetectionState::dontKnow ? 1 : whistleDortmund.lastDetectionTime){};
  ,
  (unsigned int)(0) lastDetectionTime
);
