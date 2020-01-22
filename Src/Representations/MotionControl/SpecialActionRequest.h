/**
 * @file Representations/MotionControl/SpecialActionRequest.h
 * This file declares a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

 /**
  * @struct SpecialActionRequest
  * The struct represents special action requests.
  */
STREAMABLE(SpecialActionRequest,
{
  /** ids for all special actions */
  ENUM(SpecialActionID,
  {,
    playDead,
    sitDown,
    stand,
    standHigh,
    // add new non-standup and non-block motions here
    numOfBasicMotions,
    // -------- from here on, gyro fall down detection is off for all motions               ---------
    standUpBackNao = numOfBasicMotions,
    standUpFrontNao,
    numOfStandUpMotions,
    standUpSideNao = numOfStandUpMotions,
    // block motions
    // add new block motions here
    // -------- up to numOfSpecialActionIDs, gyro fall down detection is off for all motions ---------
    // -------- so do not add non-block motions here!
    // penalty shootout block motions
  });

/**
 * The function searches the id for a special action name.
 * @param name The name of the special action.
 * @return The corresponding id if found, or numOfSpecialActions if not found.
 */
static SpecialActionID getSpecialActionFromName(const char* name),

  (SpecialActionID)(playDead) specialAction, /**< The special action selected. */
  (bool)(false) mirror, /**< Mirror left and right. */
});
