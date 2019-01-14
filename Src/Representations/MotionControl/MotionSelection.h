/**
 * @file Representations/MotionControl/MotionSelection.h
 * This file declares a struct that represents the motions actually selected based on the constraints given.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "MotionRequest.h"
#include <cstring>

/**
 * @struct MotionSelection
 * A struct that represents the motions actually selected based on the constraints given.
 */
STREAMABLE(MotionSelection,
{
  ENUM(ActivationMode,
  {,
    deactive,
    active,
    first,
  });

  /** Special action is the default selection. */
  MotionSelection()
  {
    memset(ratios, 0, sizeof(ratios));
    memset(timeStampLastExecuted, 0, sizeof(timeStampLastExecuted));
    ratios[MotionRequest::specialAction] = 1;
  },

  ((MotionRequest) Motion)(specialAction) targetMotion, /**< The motion that is the destination of the current interpolation. */
  (ActivationMode)(active) specialActionMode, /**< Whether and how the special action module is active. */
  (float[MotionRequest::numOfMotions]) ratios, /**< The current ratio of each motion in the final joint request. */
  (unsigned[MotionRequest::numOfMotions]) timeStampLastExecuted, /**< The last time the motion was active (i.e. ratio > 0). */
  (SpecialActionRequest) specialActionRequest, /**< The special action request, if it is an active motion. */
  (WalkRequest) walkRequest, /**< The walk request, if it is an active motion. */
});
