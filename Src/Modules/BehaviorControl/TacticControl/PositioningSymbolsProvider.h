/**
* @file PositioningSymbolsProvider.h
*
* Declaration of class PositioningSymbolsProvider.
* Provides the optimal positions for all players in PLAY and READY dependent on their role.
* All positions are in world coordinates with a positioning angle in degrees (for CABSL).
*
* @author <a href="mailto:janine.frickenschmidt@tu-dortmund.de">Janine Frickenschmdit</a>
*/

#pragma once

#include "Representations/BehaviorControl/BallSearch.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderLeft.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderRight.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderSingle.h"
#include "Representations/BehaviorControl/RoleSymbols/Keeper.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/RoleSymbols/ReplacementKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/LeftWing.h"
#include "Representations/BehaviorControl/RoleSymbols/RightWing.h"
#include "Representations/BehaviorControl/RoleSymbols/FrontWing.h"
#include "Representations/BehaviorControl/RoleSymbols/BackWing.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"

MODULE(PositioningSymbolsProvider,
  REQUIRES(RoleSymbols),
  REQUIRES(BallSearch),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(FieldDimensions),
  REQUIRES(GameSymbols),
  REQUIRES(DefenderRight),
  REQUIRES(DefenderLeft),
  REQUIRES(Center),
  REQUIRES(Keeper),
  REQUIRES(Ballchaser),
  REQUIRES(DefenderSingle),
  REQUIRES(Receiver),
  REQUIRES(BackupBallchaser),
  REQUIRES(ReplacementKeeper),
  REQUIRES(LeftWing),
  REQUIRES(RightWing),
  REQUIRES(BackWing),
  REQUIRES(FrontWing),
  REQUIRES(GameInfo),
  REQUIRES(BallChaserDecision),
  REQUIRES(RobotInfo),

  PROVIDES(PositioningSymbols),
  PROVIDES(PositioningAndKickSymbols)
);


/**
* @class PositioningSymbolsProvider
* Symbols for behavior 2019
*/

class PositioningSymbolsProvider : public PositioningSymbolsProviderBase
{
public:
  /** Constructor */
  PositioningSymbolsProvider() {}

  /** Updates some of the symbols */
  void update(PositioningSymbols& positioningSymbols) override;
  void update(PositioningAndKickSymbols& positioningAndKickSymbols) override;

private:
  void selectClosestBallSearchPosition(const BallSearch::BallSearchPositions& ballSearchPositions, Pose2f& position);

  int ballSearchPointIndex = 0;
};
