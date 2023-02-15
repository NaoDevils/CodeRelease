/**
* @file RoleDynamicProvider.h
*
* Declaration of class RoleDynamicProvider.
* The dynamic role provider makes the assignment from robot number to role. Possible roles are decided by the current tactic.
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderLeft.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderRight.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderSingle.h"
#include "Representations/BehaviorControl/RoleSymbols/Keeper.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/RoleSymbols/ReplacementKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/DangerMap.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/BehaviorControl/RoleSymbols.h"

namespace NDBehavior
{
  ENUM(RoleAssignmentType,
    minDistanceSum,
    minWalkingDistance,
    minScore,
    minDegreeDistance
  );
}

MODULE(RoleDynamicProvider,
  REQUIRES(BallChaserDecision),
  REQUIRES(BallSymbols),
  REQUIRES(Ballchaser),
  REQUIRES(BackupBallchaser),
  REQUIRES(Center),
  REQUIRES(DefenderLeft),
  REQUIRES(DefenderRight),
  REQUIRES(DefenderSingle),
  REQUIRES(Keeper),
  REQUIRES(Receiver),
  REQUIRES(ReplacementKeeper),
  REQUIRES(RoleSelection),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateData),
  REQUIRES(DangerMap),
  REQUIRES(RobotMap),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(FallDownState),
  REQUIRES(OwnTeamInfo),
  PROVIDES(RoleSymbols),
  DEFINES_PARAMETERS(,
    (bool)(true) useStaticAssignmentNoWifi
  )
);

/**
* @class RoleDynamicProvider
* Symbols for role decision
*/
class RoleDynamicProvider : public RoleDynamicProviderBase
{
public:
  /** Constructor */
  RoleDynamicProvider()
  {
    passiveRolePositions.clear();
    robotPoses.clear();
    for (int i = 0; i <= MAX_NUM_PLAYERS; i++)
    {
      BehaviorData::PassiveRolePositionVector prpv;
      passiveRolePositions.push_back(prpv);
      robotPoses.emplace_back(Pose2f());
    }
  }

private:
  /** Updates some of the symbols */
  void update(RoleSymbols& roleSymbols);

  /** Find the player numbers and current positions of the active players in the team. */
  void getCurrentTeamStatus();

  /** Declare drawing for the role selection in the simulator. */
  void declareDebugDrawing(RoleSymbols& roleSymbols);

  /**
   * @brief In case of no wifi, static assignment except for ball chaser
   * @param roleSymbols 
  */
  void setStaticAssignment(RoleSymbols& roleSymbols);

  void addRoles(RoleSymbols& roleSymbols);

  void findBestRoleAssignment(RoleSymbols& roleSymbols);

  void addMyPassiveRolePositions();

  void getPositionForRoleAndPlayer(const BehaviorData::RoleAssignment role, const int playerNumber, Vector2f& position);

  void takeNewestRoleAssignment(RoleSymbols& roleSymbols);

  std::vector<BehaviorData::RoleAssignment> roles; // roles that should be assigned in role selection
  std::vector<BehaviorData::PassiveRolePositionVector> passiveRolePositions; // collecting all passive role suggestions
  std::vector<int> playerNumbers; // all player numbers in the pool for role assignment
  std::vector<Pose2f> robotPoses; // all player positions
  int lastPlayerNumberToBall = 0;
};
