#include "RoleSelectionProvider.h"
#include <algorithm>

RoleSelectionProvider::RoleSelectionProvider()
{
  const auto verify = [](const std::array<RoleSelection, MAX_NUM_PLAYERS - 1>& selection)
  {
    for (size_t i = 0; i < selection.size(); ++i)
    {
      ASSERT(selection[i].selectedRoles.size() == i + 1);
      ASSERT(std::find(selection[i].selectedRoles.begin(), selection[i].selectedRoles.end(), selection[i].ballchaserDuringOppKickoff) != selection[i].selectedRoles.end());
      ASSERT(std::find(selection[i].selectedRoles.begin(), selection[i].selectedRoles.end(), selection[i].ballchaserDuringOwnKickoff) != selection[i].selectedRoles.end());
      ASSERT(std::find(selection[i].selectedRoles.begin(), selection[i].selectedRoles.end(), BehaviorData::RoleAssignment::noRole) == selection[i].selectedRoles.end());
    }
  };
  verify(defensive);
  verify(offensive);
}

void RoleSelectionProvider::update(RoleSelection& roleSelection)
{
  selectRoles(roleSelection);
  // check if our own kickoff is in process
  ownKickoffInProcess(roleSelection);
  // if we have kickoff deploy receiver
  deployReceiverInKickoff(roleSelection);
  return;
}

/**
* \brief Chooses the correct fundamental role selection from the config file.
*
* @param numberOfFieldPlayers Number of field players that need roles.
* @param defensiveBehavior Should the more defensive role selection be chosen?
* @param closeToOwnGoal Is the ball located in front of the OffenseLine?
*/
RoleSelection RoleSelectionProvider::makeFundamentalSelection(int numberOfFieldPlayers, bool defensiveBehavior)
{
  const auto& selection = defensiveBehavior ? defensive : offensive;
  if (numberOfFieldPlayers < 1 || numberOfFieldPlayers > static_cast<int>(selection.size()))
  {
    OUTPUT_ERROR("RoleSelectionProvider: Invalid number of field players!");
    return selection.back();
  }
  return selection[numberOfFieldPlayers - 1];
}

/**
* \brief Removes a role from the role selection and replaces it with another.
*
* If the role that needs to be replaced is not found in the role selection nothing will be changed.
* The method will return a boolean indicating if it executed the substitution.
*
* @param toReplace A role that is included in the role selection and needs to be replaced.
* @param substitute The role to insert after the other is removed.
* @param roleSelection The role selection on which the switch is executed.
*
* @returns True if the switch took place (i.e. toReplace was found in roleSelection), false otherwise.
*/
bool RoleSelectionProvider::substituteRoles(BehaviorData::RoleAssignment toReplace, BehaviorData::RoleAssignment substitute, BehaviorData::RobotRoleAssignmentVector& roleSelection)
{
  // find the role to replace and remove it
  BehaviorData::RobotRoleAssignmentVector::iterator position = std::find(roleSelection.begin(), roleSelection.end(), toReplace);
  if (position != roleSelection.end())
    roleSelection.erase(position);
  else
    return false;
  // add new role
  roleSelection.push_back(substitute);
  return true;
}

/**
* \brief Checks if the keeper is active and defends the goal.
*
* The keeper is treated as not available if it is not active (e.g. penalized)
*/
bool RoleSelectionProvider::isKeeperAvailable()
{
  // the keeper needs to check its own penalty seperately because it is not in its own teammateData
  if (theRobotInfo.number == 1)
    return theRobotInfo.penalty == PENALTY_NONE;

  if (theTeammateData.commEnabled)
  {
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.playerNumber == 1 && mate.status >= TeammateReceived::Status::ACTIVE)
        return true;
    }
    // if no active keeper has been found return false
    return false;
  }
  // Fallback to game controller data
  else if (theGameInfo.controllerConnected)
  {
    return theOwnTeamInfo.players[0].penalty == PENALTY_NONE;
  }
  // Fallback to 7 players
  else
  {
    return true;
  }
}

/**
* \brief Checks the keeper needs to be replaced by a field player.
*
* Currently only checks if keeper is unavailable and there are multiple field players.
* A single field player chasing the ball would be more benefitial then a replacementKeeper.
*/
bool RoleSelectionProvider::isReplacementKeeperNeeded(RoleSelection& roleSelection)
{
  bool replacementKeeperNeeded = !isKeeperAvailable() && theTacticSymbols.numberOfActiveFieldPlayers > 1 && enableReplacementKeeper;
  return replacementKeeperNeeded;
}

/**
* \brief Checks if a role is present in a role selection.
*
* \param roleSelection Check presence of role in this role selection.
* \param role The role to search for 
*
* \return True if role is present in the role selection, false otherwise.
*/
bool RoleSelectionProvider::isRoleInSelection(BehaviorData::RobotRoleAssignmentVector& roleVector, BehaviorData::RoleAssignment role)
{
  BehaviorData::RobotRoleAssignmentVector::iterator roleIndex = std::find(roleVector.begin(), roleVector.end(), BehaviorData::receiver);
  bool roleFound = roleIndex != roleVector.end();
  return roleFound;
}

/**
* \brief Fill role selection with the missing number of roles.
*
* Which roles are used to fill the selection is determined by the roleSelectionProvider.cfg.
*/
void RoleSelectionProvider::fillRoleSelection(RoleSelection& roleSelection, int rolesNeeded)
{
  if (rolesNeeded > 0)
  {
    RoleSelection fundamentalSelection = makeFundamentalSelection(rolesNeeded, theTacticSymbols.defensiveBehavior);
    // append selected roles
    roleSelection.selectedRoles.insert(roleSelection.selectedRoles.end(), fundamentalSelection.selectedRoles.begin(), fundamentalSelection.selectedRoles.end());
    roleSelection.ballchaserDuringOppKickoff = fundamentalSelection.ballchaserDuringOppKickoff;
    roleSelection.ballchaserDuringOwnKickoff = fundamentalSelection.ballchaserDuringOwnKickoff;
  }
  else
  {
    roleSelection.ballchaserDuringOppKickoff = BehaviorData::RoleAssignment::noRole;
    roleSelection.ballchaserDuringOwnKickoff = BehaviorData::RoleAssignment::noRole;
  }
}

/**
* \brief Choose fundamental alignment and tweak it based on the current situation.
*
* Selecting the roles that are needed right now is done in two steps-
* First one of the predefined (fundamental) role selections given in the config is chosen
* based on two factors:
*     - number of active field players
*     - tactical alignment of the team ("defensive" or "offensive")
*
* The chosen role selection will the be tweaked based on the current situation.
*/
void RoleSelectionProvider::selectRoles(RoleSelection& roleSelection)
{
  roleSelection.selectedRoles.clear();

  int rolesNeeded = theTacticSymbols.numberOfActiveFieldPlayers;
  if (isKeeperAvailable())
    roleSelection.selectedRoles.push_back(BehaviorData::keeper);
  else if (isReplacementKeeperNeeded(roleSelection))
  {
    // if field player needs to replace keeper => deploy replacementKeeper role reduce rolesNeeded
    roleSelection.selectedRoles.push_back(BehaviorData::replacementKeeper);
    rolesNeeded--;
  }
  // fill your selection with as many roles as needed to complete the team
  fillRoleSelection(roleSelection, rolesNeeded);
}

/**
* \brief Checks if the own kickoff is currently executed.
*
* Our own kickoff includes the ready and set phase before the kick off as well as the first push
* after the kickoff. Our kickoff ends if:
*     - the opponent team gets a kickoff
*     - the ball moves too far into our own half meaning the first attack was unsuccesful
* The method sets attributes of the RoleSelectionProvider since the results of this frame are needed in
* the calculations of the next one.
*
* \return True if own kickoff is in process, false otherwise.
*/
bool RoleSelectionProvider::ownKickoffInProcess(RoleSelection& RoleSelection)
{
  bool inReadyOrSet = theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET;
  bool beforeOwnKickoff = inReadyOrSet && theGameSymbols.ownKickOff;
  float terminationLine = 0.3f * theFieldDimensions.xPosOwnGroundline;
  bool ballIsInOwnDefense = theBallSymbols.ballPositionField.x() < terminationLine;

  inOwnKickoffSituation = beforeOwnKickoff || (inOwnKickoffSituation && !ballIsInOwnDefense && theGameSymbols.ownKickOff);
  return inOwnKickoffSituation;
}

/** 
* \brief Deploy receiver under certain conditions when own team performs kickoff.
*
* When the own team has kickoff and enough robots are in play a receiver should be temporarily 
* deployed, if it is not already on the field. The receiver will stay active until the attack after
* the kickoff ended. Currently the receiver will replace the backupBallchaser if active or the 
* center otherwise.
*/
void RoleSelectionProvider::deployReceiverInKickoff(RoleSelection& roleSelection)
{
  // check if role selection needs to be changed
  bool receiverIsActive = isRoleInSelection(roleSelection.selectedRoles, BehaviorData::receiver);
  // use receiver as long as ball did not roll to far into our half after own kickoff
  bool needReceiver = !receiverIsActive && inOwnKickoffSituation && theTacticSymbols.numberOfActiveFieldPlayers > 3 && forceReceiverOnOwnKickoff;
  if (needReceiver)
  {
    // try to replace the backupBallChaser
    bool backupSubstituteSuccessful = substituteRoles(BehaviorData::backupBallchaser, BehaviorData::receiver, roleSelection.selectedRoles);
    // if backupBallchaser is not active try to replace the center
    if (!backupSubstituteSuccessful)
      substituteRoles(BehaviorData::center, BehaviorData::receiver, roleSelection.selectedRoles);
  }
}

MAKE_MODULE(RoleSelectionProvider, behaviorControl)
