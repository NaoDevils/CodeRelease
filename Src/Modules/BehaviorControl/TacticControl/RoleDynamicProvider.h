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
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/RoleSymbols/BackupBallchaser.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderLeft.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderRight.h"
#include "Representations/BehaviorControl/RoleSymbols/DefenderSingle.h"
#include "Representations/BehaviorControl/RoleSymbols/Keeper.h"
#include "Representations/BehaviorControl/RoleSymbols/Receiver.h"
#include "Representations/BehaviorControl/RoleSymbols/ReplacementKeeper.h"
#include "Representations/BehaviorControl/RoleSymbols/LeftWing.h"
#include "Representations/BehaviorControl/RoleSymbols/RightWing.h"
#include "Representations/BehaviorControl/RoleSymbols/FrontWing.h"
#include "Representations/BehaviorControl/RoleSymbols/BackWing.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/RobotPose.h"
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
  REQUIRES(BackupBallchaser),
  REQUIRES(Center),
  REQUIRES(DefenderLeft),
  REQUIRES(DefenderRight),
  REQUIRES(DefenderSingle),
  REQUIRES(LeftWing),
  REQUIRES(RightWing),
  REQUIRES(FrontWing),
  REQUIRES(BackWing),
  REQUIRES(Keeper),
  REQUIRES(Receiver),
  REQUIRES(ReplacementKeeper),
  REQUIRES(RoleSelection),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateData),
  REQUIRES(RobotPoseAfterPreview),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TacticSymbols),
  PROVIDES(RoleSymbols),
  LOADS_PARAMETERS(,
    (bool)(true) useStaticAssignmentNoWifi,
    (float)(1000.f) minDistanceDiffForNewRoleAssignment
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
  RoleDynamicProvider() { rolePositions.fill(Vector2f::Zero()); }

private:
  /** Updates some of the symbols */
  void update(RoleSymbols& roleSymbols);

  /** Find the player numbers and current positions of the active players in the team. */
  void getCurrentTeamStatus();

  /**
   * @brief In case of no wifi, static assignment except for ball chaser
   * @param roleSymbols 
  */
  void setStaticAssignment(RoleSymbols& roleSymbols);

  bool roleChangeNeccessary(RoleSymbols& roleSymbols);

  void addRoles(RoleSymbols& roleSymbols);

  void findBestRoleAssignment(RoleSymbols& roleSymbols);

  void compareToLastAssignment(RoleSymbols& roleSymbols, const std::vector<float>& bestWalkDistances);

  void takeNewestRoleAssignment(RoleSymbols& roleSymbols);

  void updateRolePositions();

  std::vector<BehaviorData::RoleAssignment> roles; // roles that should be assigned in role selection
  std::array<Vector2f, BehaviorData::RoleAssignment::numOfRoleAssignments> rolePositions;
  std::vector<int> playerNumbers; // all player numbers in the pool for role assignment
  std::array<Pose2f, MAX_NUM_PLAYERS + 1> robotPoses; // all player positions
};
