/**
* @file BallSearchProvider.h
*
* Declaration of class BallSearchProvider.
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallSearch.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotMap.h"

MODULE(BallSearchProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(RoleSelection),
  REQUIRES(TeammateData),
  REQUIRES(RobotMap),
  REQUIRES(RoleSymbols),
  REQUIRES(BallChaserDecision),
  REQUIRES(RobotInfo),
  PROVIDES(BallSearch),
  LOADS_PARAMETERS(,
    (int)(10000) timeUntilWholeFieldSearchAfterLost,
    (float)(0.2f) defenseSearchStart,
    (float)(0.45f) centerSearchStart
  )
);


/**
* @class BallSearchProvider
* Symbols for new role behavior 2019
*/

class BallSearchProvider : public BallSearchProviderBase
{
public:
  /** Constructor */
  BallSearchProvider() {}

  /** Updates some of the symbols */
  void update(BallSearch& ballsearch);

private:
  void fillBallSearchPositions(BallSearch& ballsearch);

  // members
  unsigned int timeStampBallLostForTeam = 0;
  int defenseInBallSearch = 0;
  int centerInBallSearch = 0;
  int offenseInBallSearch = 0;
  Vector2f lastBallPositionField = Vector2f::Zero();
};
