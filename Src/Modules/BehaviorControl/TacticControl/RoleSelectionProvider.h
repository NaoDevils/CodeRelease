/**
* @file RoleSelectionProvider.h
*
* Declaration of class RoleSelectionProvider.
* Determines the roles that used be used in the current situation.
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Modules/BehaviorControl/CABSL/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/BehaviorControl/RoleSelection.h"
#include "Representations/BehaviorControl/TacticSymbols.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Settings.h"

MODULE(RoleSelectionProvider,
  REQUIRES(BallSymbols),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(GameSymbols),
  REQUIRES(TacticSymbols),
  PROVIDES(RoleSelection),
  LOADS_PARAMETERS(
    ,
    (bool)(true) forceReceiverOnOwnKickoff,
    (bool)(true) enableReplacementKeeper,
    (std::array<RoleSelection, MAX_NUM_PLAYERS-1>) defensive,
    (std::array<RoleSelection, MAX_NUM_PLAYERS-1>) offensive
  )
);

/**
* @class RoleSelectionProvider
* Decides which roles should be deployed.
*/
class RoleSelectionProvider : public RoleSelectionProviderBase
{
public:
  /** Constructor */
  RoleSelectionProvider();

private:
  /** Updates some of the symbols */
  void update(RoleSelection& roleSelection);

  /** Take the fundamental role selection from the config based on various factors*/
  RoleSelection makeFundamentalSelection(int numberOfFieldPlayers, bool defensiveBehavior);

  /** Removes a role from the role selection and replaces it with another. */
  bool substituteRoles(BehaviorData::RoleAssignment toReplace, BehaviorData::RoleAssignment substitute, BehaviorData::RobotRoleAssignmentVector& roleSelection);

  /** Checks if the keeper is active and defends the goal. */
  bool isKeeperAvailable();

  /** Checks the keeper needs to be replaced by a field player. */
  bool isReplacementKeeperNeeded(RoleSelection& roleSelection);

  /** Checks wether a given role is already selected in a role selection.*/
  bool isRoleInSelection(BehaviorData::RobotRoleAssignmentVector& roleSelection, BehaviorData::RoleAssignment role);

  /** Fills role selection with missing number of roles */
  void fillRoleSelection(RoleSelection& roleSelection, int rolesNeeded);

  /** Decide which roles should participate in the current situation. */
  void selectRoles(RoleSelection& roleSelection);

  /** Check if we are currently executing our kick off (includes the attack after the kickoff). */
  bool ownKickoffInProcess(RoleSelection& roleSelection);

  /** Tweak role selection when own team performs kickoff. */
  void deployReceiverInKickoff(RoleSelection& roleSelection);

  // vvv--- Attributes used to keep track of calculation results over several frames. ---vvv
  bool inOwnKickoffSituation = false;
};
