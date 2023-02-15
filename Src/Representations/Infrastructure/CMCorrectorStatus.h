/**
 * @file CMCorrectorStatus.h
 * Declaration of a struct for representing the camera matrix' calibration status.
 * @author Aaron Larisch
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

STREAMABLE(CMCorrectorStatus,
  ENUM(CalibrationState,
    inactive,
    positioning,
    stand,
    wait,
    captureUpper,
    captureLower,
    optimizeUpper,
    optimizeLower,
    finished
  );
  ,
  (CalibrationState)(CalibrationState::inactive) state,
  (unsigned char)(0) stage,
  (float)(0.f) progress
);
