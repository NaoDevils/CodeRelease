/**
* @file SpeedInfo 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include <limits>
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Representations/MotionControl/WalkRequest.h"
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"
#else
#include "bhumanstub.h"
#endif

/**
 * @class SpeedInfo
 * Contains information about the actual speed.
 */
STREAMABLE(SpeedInfo,,
  (Pose2f) speed, /**< The< actual speed. */
  (Pose2f) speedAfterPreview, /**< Future speed in ControllerParams::N frames */
  (bool)(false) deceleratedByAcc, /** Decelerated due to limited acceleration */
  (int)(0) timestamp, /** Timestamp of the foot executing this speed */

  /** Currently executed custom step file. If this is not "none" the speed above
   is invalid. */
  ((WalkRequest) StepRequest)(none) currentCustomStep,

  ((WalkRequest) StepRequest)(none) lastCustomStep,
  (bool)(false) lastCustomStepMirrored,
  (unsigned)(0) lastCustomStepTimestamp,

  (bool)(false) customStepKickInPreview,
  (int)(0) stepsSinceLastCustomStep
);

STREAMABLE(SpeedInfoCompressed,
  // Increase version number whenever something changes!
  static constexpr unsigned char version = 0;

  SpeedInfoCompressed() = default;
  explicit SpeedInfoCompressed(const SpeedInfo& other)
   : speed(other.speed)
  {
    // Convert to mm
    speed.translation *= 1000.f;
  }
  ,
  (Pose2fCompressed) speed
);
