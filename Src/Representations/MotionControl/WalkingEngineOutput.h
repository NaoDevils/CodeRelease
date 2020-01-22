/**
 * @file Representations/MotionControl/WalkingEngineOutput.h
 * This file declares a struct that represents the output of modules generating motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/WalkRequest.h"

/**
 * @struct WalkingEnigeOutput
 * A struct that represents the output of the walking engine.
 */
STREAMABLE_WITH_BASE(WalkingEngineOutput, JointRequest,
{,
  (bool)(true) standing, /**< Whether the robot is standing or walking */
  (Pose2f) speed, /**< The current walking speed in mm/s and rad/s. */
  (Pose2f) odometryOffset, /**< The body motion performed in this step. */
  (Pose2f) offsetToRobotPoseAfterPreview, /**< The offset after execution of preview phase. */
  (bool)(true) isLeavingPossible, /**< Is leaving the motion module possible now? */
  (float)(0) positionInWalkCycle, /**< The current position in the walk cycle in the range [0..1[. */
  (WalkRequest) executedWalk, /**< The walk currently executed. */
});
