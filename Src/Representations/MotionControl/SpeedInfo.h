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
  SpeedInfoCompressed() = default;
  SpeedInfoCompressed(const SpeedInfo& other)
  {
    translation = other.speed.translation.cast<short>();
    rotation = static_cast<unsigned char>((other.speed.rotation + pi) / pi2 * std::numeric_limits<unsigned char>::max());
    currentCustomStep = other.currentCustomStep;
  }

  operator SpeedInfo() const
  {
    SpeedInfo info;
    info.speed.translation = translation.cast<float>();
    info.speed.rotation = static_cast<float>(rotation) / pi2 - pi;
    info.currentCustomStep = currentCustomStep;
    return info;
  },

  (Vector2s) translation,
  (unsigned char)(0) rotation,
  ((WalkRequest) StepRequest)(WalkRequest::none) currentCustomStep
);
