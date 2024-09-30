#pragma once

#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"

STREAMABLE(WhistleDortmund,
  ENUM(DetectionState,
        dontKnow,
        isKnown,  
        notDetected,
        isDetected
  )

  ENUM(Microphone,
        rearLeft,
        rearRight,
        frontLeft,
        frontRight
  )
  ENUM(WhistleDetectionStatus,
        alright,
        tryLocalization,
        onlyWhlistleDetection,
        broken
  ),

  (WhistleDetectionStatus) (alright) whistleDetectionStatus,
  (DetectionState)(dontKnow) detectionState, /**< Was detected? */
  (unsigned int)(0) lastDetectionTime,
  (int)(0) detectedWhistleFrequency,
  (int)(0) minFrequency,
  (int)(0) maxFrequency,
  (unsigned int)(0) lastWhistleLength,
  (float)(0.f) lastConfidence,
  (DetectionState)(dontKnow) directionState,
  (Angle)(0_deg) rawDirection,
  (Angle)(0_deg) correctedDirection,
  (DetectionState)(dontKnow) distanceState,
  (float)(0.f) distance,
  (std::string)("") readableDistance,
  (std::array<float, 5>) distanceConfidence,
  (std::array<bool, 4>) micStatus
);

STREAMABLE(WhistleDortmundCompressed,
  // Increase version number whenever something changes!
  static constexpr unsigned char version = 1;

  WhistleDortmundCompressed() = default;
  explicit WhistleDortmundCompressed(const WhistleDortmund& whistleDortmund)
  {
    lastDetectionTime = whistleDortmund.detectionState == WhistleDortmund::DetectionState::dontKnow ? 1 : whistleDortmund.lastDetectionTime;
   // direction = whistleDortmund.correctedDirection;
  };
  ,
  (unsigned int)(0) lastDetectionTime//,
  //(AngleCompressed)(90_deg) direction
);
