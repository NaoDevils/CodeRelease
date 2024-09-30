/**
* @file RoleDynamicProvider.cpp
*
* Implementation of class RoleDynamicProvider.
*
*/

#include "RoleDynamicProvider.h"
#include "Representations/Infrastructure/TeamCommEvents.h"
#include "Tools/Debugging/Annotation.h"
#include <algorithm>

void RoleDynamicProvider::update(RoleSymbols& roleSymbols)
{
  DECLARE_DEBUG_DRAWING3D("representation:RoleSymbols", "robot");
  DECLARE_DEBUG_DRAWING3D("representation:RoleAssignment", "field");
  BehaviorData::RoleAssignment lastRole = roleSymbols.role;
  roleSymbols.roleSuggestions.assign(MAX_NUM_PLAYERS + 1, BehaviorData::RoleAssignment::noRole);
  roleSymbols.dynamic = false;

  updateRolePositions();

  if (theRobotInfo.penalty != PENALTY_NONE)
    return;

  if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    roleSymbols.role = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? BehaviorData::RoleAssignment::receiver : BehaviorData::RoleAssignment::keeper;
    roleSymbols.roleSuggestions[theRobotInfo.number] = roleSymbols.role;
  }
  else if (!theTeammateData.commEnabled && useStaticAssignmentNoWifi)
  {
    setStaticAssignment(roleSymbols);
  }
  else
  {
    addRoles(roleSymbols);

    getCurrentTeamStatus();

    findBestRoleAssignment(roleSymbols);

    takeNewestRoleAssignment(roleSymbols);

    roleSymbols.dynamic = true;
  }

  if (roleSymbols.role != lastRole)
    roleSymbols.lastRole = lastRole;

  ASSERT(roleSymbols.role < BehaviorData::numOfRoleAssignments);

  COMPLEX_DRAWING3D("representation:RoleAssignment")
  {
    if (const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned))
    {
      const auto& suggestions = teammate->behaviorData.roleSuggestions;
      if (teammate->playerNumber == theRobotInfo.number)
      {
        for (unsigned char player = 1; player < suggestions.size(); ++player)
        {
          if (suggestions[player] == BehaviorData::RoleAssignment::keeper || suggestions[player] == BehaviorData::RoleAssignment::noRole)
            continue;

          const ColorRGBA color = theOwnTeamInfo.teamNumber == 1 ? ColorRGBA::blue : ColorRGBA::red;

          const Vector2f drawOffset(10.f, 10.f); //add an small offset for drawing, otherwise the yellow path lines will may be visible
          const Vector2f currentPosition = robotPoses[player].translation + drawOffset;
          const Vector3f currentPosition3f(currentPosition.x(), currentPosition.y(), 0.f);

          DRAWDIGIT3D("representation:RoleAssignment", player, currentPosition3f + Vector3f(0.f, 0.f, 50.f), 20, 2, color);

          const Vector2f rolePassivePosition = rolePositions[suggestions[player]] + drawOffset;
          const Vector3f rolePassivePosition3f(rolePassivePosition.x(), rolePassivePosition.y(), 0.f);

          LINE3D("representation:RoleAssignment",
              currentPosition3f.x(),
              currentPosition3f.y(),
              currentPosition3f.z(),
              rolePassivePosition3f.x(),
              rolePassivePosition3f.y(),
              rolePassivePosition3f.z(),
              2.f,
              color);
          const Vector3f diff = (rolePassivePosition3f - currentPosition3f).normalized();
          CYLINDERARROW3D("representation:RoleAssignment", rolePassivePosition3f - diff * 50.f, rolePassivePosition3f + diff * 50.f, 2.f, 90.f, 25.f, color);
        }
      }
    }
  }
}

/**
 * \brief Collect the numbers and current positions of all active members in the team.
 * 
 * The method fills two vector with the collected values, one for the numbers of active players and
 * one for the current poses of those players. The pose vector will be indexed with the player
 * number vector, i.e. the pose in the n-th position of robotPoses belongs to the robot with the
 * number in the n-th position of playerNumbers.
 * 
 * \param roleSymbols A copy of the current RoleSymbols representation.
 */
void RoleDynamicProvider::getCurrentTeamStatus()
{
  playerNumbers.clear();

  // get information for myself
  const bool iAmKeeper = theRobotInfo.number == 1;
  const bool inReady = (theGameInfo.inPreGame() || theGameInfo.state == STATE_READY);
  const bool iAmBallchaserInReady = inReady && theRobotInfo.number == theBallChaserDecision.playerNumberToBall;
  if (!iAmKeeper && !iAmBallchaserInReady && theRobotInfo.penalty == PENALTY_NONE)
    playerNumbers.push_back(theRobotInfo.number);
  robotPoses[theRobotInfo.number] = theRobotPoseAfterPreview;

  // get information for teammates
  for (auto& mate : theTeammateData.teammates)
  {
    const bool mateIsKeeper = mate.playerNumber == 1;
    const bool mateIsBallchaserInReady = inReady && mate.playerNumber == theBallChaserDecision.playerNumberToBall;
    if (!mateIsKeeper && !mateIsBallchaserInReady && mate.status >= TeammateReceived::Status::ACTIVE)
      playerNumbers.push_back(mate.playerNumber);
    robotPoses[mate.playerNumber] = mate.robotPose;
  }
}

void RoleDynamicProvider::setStaticAssignment(RoleSymbols& roleSymbols)
{
  // fill players from lowest number to highest
  addRoles(roleSymbols);

  robotPoses.fill(Pose2f());

  auto role = roles.begin();
  for (unsigned char player = 1; player <= MAX_NUM_PLAYERS; ++player)
  {
    if (theRoleSelection.selectedRoles.size() != MAX_NUM_PLAYERS && theOwnTeamInfo.players[player - 1].penalty != PENALTY_NONE)
      continue;

    if (roleSymbols.roleSuggestions[player] == BehaviorData::RoleAssignment::noRole)
    {
      ASSERT(role != roles.end());
      roleSymbols.roleSuggestions[player] = *role;
      ++role;
    }
  }
  if (role != roles.end())
  {
    OUTPUT_WARNING("RoleDynamicProvider: There a more roles available than players for static assignment!");
  }

  roleSymbols.role = roleSymbols.roleSuggestions[theRobotInfo.number];
}

void RoleDynamicProvider::addRoles(RoleSymbols& roleSymbols)
{
  roles.clear();
  const bool inReady = theGameInfo.inPreGame() || theGameInfo.state == STATE_READY;
  const BehaviorData::RoleAssignment ballchaserRole = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? theRoleSelection.ballchaserDuringOwnKickoff : theRoleSelection.ballchaserDuringOppKickoff;
  for (const auto& role : theRoleSelection.selectedRoles)
  {
    // keeper is only role not dynamically assigned
    if (role == BehaviorData::keeper)
      roleSymbols.roleSuggestions[1] = role;
    else if (inReady && role == ballchaserRole && theBallChaserDecision.playerNumberToBall != 1)
      roleSymbols.roleSuggestions[theBallChaserDecision.playerNumberToBall] = role;
    else
      roles.push_back(role);
  }
}

void RoleDynamicProvider::findBestRoleAssignment(RoleSymbols& roleSymbols)
{
  int playerCount = static_cast<int>(roles.size());
  if (playerCount < 1)
    return; // nothing to do; i.e. only keeper was assigned
  if (playerNumbers.size() != roles.size())
  {
    OUTPUT_ERROR("RoleDynamicProvider: role number (" << playerNumbers.size() << ") and player count (" << playerCount << ") different!");
    ANNOTATION("RoleDynamicProvider", "Role number and player count differ!");
    return;
  }
  std::vector<int> optAssignment(playerNumbers);
  std::vector<float> bestWalkDistances(playerCount, 100000.f);
  // get total number of possible combinations of assignments
  int possibilities = factorial(playerCount);

  for (int i = 0; i < possibilities; i++)
  {
    // get next permutation of possible player to position assignments
    std::next_permutation(playerNumbers.begin(), playerNumbers.end());

    std::vector<float> currentWalkDistances(playerCount, 0.f); // walking distances for this assignment
    for (int j = 0; j < playerCount; j++)
    {
      Vector2f optPosition = rolePositions[roles[j]];
      currentWalkDistances.at(j) = (robotPoses[playerNumbers[j]].translation - optPosition).norm();
    }

    std::sort(currentWalkDistances.begin(), currentWalkDistances.end(), std::greater<>());
    for (int p = 0; p < playerCount; p++)
    {
      if (currentWalkDistances.at(p) > bestWalkDistances.at(p))
        break;
      if (currentWalkDistances.at(p) < bestWalkDistances.at(p))
      {
        bestWalkDistances = currentWalkDistances;
        optAssignment = playerNumbers;
      }
    }
  }
  for (size_t i = 0; i < roles.size(); i++)
    roleSymbols.roleSuggestions[optAssignment[i]] = roles[i];

  compareToLastAssignment(roleSymbols, bestWalkDistances);
}

void RoleDynamicProvider::updateRolePositions()
{
  rolePositions[BehaviorData::RoleAssignment::keeper] = theKeeper.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::defenderRight] = theDefenderRight.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::defenderLeft] = theDefenderLeft.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::defenderSingle] = theDefenderSingle.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::backupBallchaser] = theBackupBallchaser.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::replacementKeeper] = theReplacementKeeper.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::center] = theCenter.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::receiver] = theReceiver.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::leftWing] = theLeftWing.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::rightWing] = theRightWing.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::frontWing] = theFrontWing.optPosition.translation;
  rolePositions[BehaviorData::RoleAssignment::backWing] = theBackWing.optPosition.translation;
  static_assert(BehaviorData::RoleAssignment::numOfRoleAssignments == 13, "Missing role!");
}

void RoleDynamicProvider::takeNewestRoleAssignment(RoleSymbols& roleSymbols)
{
  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newBallchaser);

  const bool isOrWasStriker = theBallChaserDecision.playerNumberToBall == theRobotInfo.number || (teammate && teammate->behaviorData.playerNumberToBall == theRobotInfo.number);

  const Teammate* teammateRoles = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned);
  if (isOrWasStriker || !teammate || !teammateRoles)
  {
    if (teammateRoles)
      roleSymbols.role = teammateRoles->behaviorData.roleSuggestions[theRobotInfo.number];
    else
      roleSymbols.role = roleSymbols.roleSuggestions[theRobotInfo.number];
  }
  else
  {
    roleSymbols.role = teammateRoles->behaviorData.roleSuggestions[theRobotInfo.number];
    roleSymbols.roleSuggestions = teammateRoles->behaviorData.roleSuggestions;
  }
}

void RoleDynamicProvider::compareToLastAssignment(RoleSymbols& roleSymbols, const std::vector<float>& bestWalkDistances)
{
  // Always use the best assignment when in initial
  if (theGameInfo.inPreGame())
    return;

  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned);

  if (!teammate)
    return;

  const auto& currentSuggestions = teammate->behaviorData.roleSuggestions;

  // check if player count is still the same
  {
    const auto hasRole = [](const BehaviorData::RoleAssignment role)
    {
      return role != BehaviorData::RoleAssignment::noRole;
    };
    const size_t numberOfRoles = std::count_if(currentSuggestions.begin(), currentSuggestions.end(), hasRole);

    if (numberOfRoles != theRoleSelection.selectedRoles.size())
      return;
  }

  // check if the same roles are assigned
  {
    const bool sameRoles = std::all_of(currentSuggestions.begin(),
        currentSuggestions.end(),
        [&](const BehaviorData::RoleAssignment role)
        {
          if (role == BehaviorData::RoleAssignment::noRole)
            return true;
          return std::find(theRoleSelection.selectedRoles.begin(), theRoleSelection.selectedRoles.end(), role) != theRoleSelection.selectedRoles.end();
        });

    if (!sameRoles)
      return;
  }

  // check if the same players are active
  {
    std::unordered_set<int> playersToCheck;
    for (const Teammate& mate : theTeammateData.teammates)
      playersToCheck.insert(mate.playerNumber);
    for (int player = 0; player < static_cast<int>(currentSuggestions.size()); ++player)
      if (currentSuggestions[player] != BehaviorData::RoleAssignment::noRole)
        playersToCheck.erase(player);

    if (!playersToCheck.empty())
      return;
  }

  // check if the correct role is replaced by the ballchaser during ready
  {
    const bool inReady = (theGameInfo.inPreGame() || theGameInfo.state == STATE_READY);
    const BehaviorData::RoleAssignment ballchaserRole = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? theRoleSelection.ballchaserDuringOwnKickoff : theRoleSelection.ballchaserDuringOppKickoff;
    if (inReady && currentSuggestions[theBallChaserDecision.playerNumberToBall] != ballchaserRole)
      return;
  }

  // check if best max distance is much better than current max distance
  std::vector<float> currentWalkDistances;
  for (unsigned char p = 0; p < currentSuggestions.size(); ++p)
  {
    if (currentSuggestions[p] != BehaviorData::RoleAssignment::noRole && currentSuggestions[p] != BehaviorData::RoleAssignment::keeper)
      currentWalkDistances.emplace_back((robotPoses[p].translation - rolePositions[currentSuggestions[p]]).norm());
  }

  const float currentMaxDistance = *std::max_element(currentWalkDistances.begin(), currentWalkDistances.end());

  // do not care about distance in ready
  if (!theTacticSymbols.keepRoleAssignment && currentMaxDistance > bestWalkDistances[0] + minDistanceDiffForNewRoleAssignment)
    return;

  roleSymbols.roleSuggestions = currentSuggestions;
}

MAKE_MODULE(RoleDynamicProvider, behaviorControl)
