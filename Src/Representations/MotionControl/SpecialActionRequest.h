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
  /** ids for all special actions */
  ENUM(SpecialActionID,
    playDead,
    sitDown,
    stand,
    test,
    testUnstiff,
    standHigh,
    cheering1,
    cheering2,
    cheering3,
    cheering4,
    cheering5,
    wave_left,
    penaltyGoaliePrepareDive,
    // add new non-standup and non-block motions here
    numOfUprightMotions,
    
    // stand up motions
    firstStandUpMotion = numOfUprightMotions,
    standUpBack = firstStandUpMotion,
    standUpBackSlide,
    standUpBackSpreadLegs,
    standUpBackLean,
    lying,
    freeArmsFront,
    freeArmsBack,
    untangleArms,
    standUpFront,
    standUpFrontSlide,
    standUpFrontSlidePart2,
    standUpFrontLean,
    standUpFrontPull,
    lastStandUpMotion,
    standUpSide = lastStandUpMotion,

    // falldown protection
    firstFallMotion,
    saveFallFront = firstFallMotion,
    saveFall,
    sit,
    rip,
    lastFallMotion,
    saveFallBack = lastFallMotion,

    // block motions
    firstBlockMotion,
    wideStanceWithStandUp = firstBlockMotion,
    goalkeeperDefendLeft,
    penaltySpeedDiveLeft,
    lastBlockMotion,
    penaltyGoalieDiveLeft = lastBlockMotion,

    // do not use any motion at all
    none
  );
using MotionIDVector = std::vector<SpecialActionID>;
/**
 * The function searches the id for a special action name.
 * @param name The name of the special action.
 * @return The corresponding id if found, or numOfSpecialActions if not found.
 */
static SpecialActionID getSpecialActionFromName(const char* name),

  (SpecialActionID)(playDead) specialAction, /**< The special action selected. */
  (bool)(false) mirror /**< Mirror left and right. */
);
