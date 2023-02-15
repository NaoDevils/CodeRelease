/**
* \file GoalSymbols.h
* The file declares a class that containts data about the current behavior state.
* \author Oliver Urbann
*/

#pragma once
#include "Tools/Streams/Streamable.h"

/**
* \class GoalSymbols
* A class that containts data about the current behavior state.
*/
STREAMABLE(GoalSymbols,,
  (Angle)(0) openingAngleOfOppGoal,
  (Angle)(0) centerAngleToOwnGoal,
  (Angle)(0) centerAngleBallToOppGoalWC,
  (Angle)(0) leftAngleBallToOppGoalWC,
  (Angle)(0) rightAngleBallToOppGoalWC,
  (Angle)(0) centerAngleToOppGoalAsSeenFromBall,
  (Angle)(0) smallestBallToGoalPostAngle,
  (Angle)(0) approachAngleWC
);
