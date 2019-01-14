/**
 * @file MotionInfo.h
 * Definition of struct MotionInfo.
 * @author Martin Lötzsch
 */

#pragma once

#include "MotionRequest.h"

/**
 * @struct MotionInfo
 * The executed motion request and additional information about the motions which are executed by the Motion process.
 */
STREAMABLE_WITH_BASE(MotionInfo, MotionRequest,
{,
  (bool)(false) isMotionStable, /**< If true, the motion is stable, leading to a valid torso / camera matrix. */
  (Pose2f) offsetToRobotPoseAfterPreview, /**< The remaining odometry offset for the currently executed motion. */
  (bool)(false) customStepKickInPreview, /**< If true, the custom step kick motion is inserted into preview */
});
