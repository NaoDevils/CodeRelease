/** 
* @file GoalSymbolsProvider.h
*
* Declaration of class GoalSymbolsProvider.
*
* @author Tim Laue
*/

#pragma once

#include <algorithm>
#include "Tools/Module/Module.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GoalSymbols.h"

MODULE(GoalSymbolsProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPoseAfterPreview),
  PROVIDES(GoalSymbols)
);

class GoalSymbolsProvider : public GoalSymbolsProviderBase
{
public:
  /*
  * Constructor.
  */
  GoalSymbolsProvider() {}

  void update(GoalSymbols& goalSymbols);

private:
  float angleToLeftOppGoalPost;
  float angleToRightOppGoalPost;
  float centerAngleToOppGoal;
  bool goalInFrontOfMe;
  bool goalInFrontOfMeSafe;
  float angleToleranceToOpponentGoal;
};
