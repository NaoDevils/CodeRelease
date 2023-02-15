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
  roleSymbols.roleSuggestions.clear();
  for (int i = 0; i <= MAX_NUM_PLAYERS; i++)
    roleSymbols.roleSuggestions.push_back(BehaviorData::noRole);

  if (theRobotInfo.penalty != PENALTY_NONE)
    return;

  if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
  {
    roleSymbols.role = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? BehaviorData::RoleAssignment::ballchaser : BehaviorData::RoleAssignment::keeper;
    roleSymbols.roleSuggestions[theRobotInfo.number] = roleSymbols.role;
  }
  else
  {
    if (!theTeammateData.wlanOK && useStaticAssignmentNoWifi)
    {
      // if no wifi, set static role using 2021 robot setup
      setStaticAssignment(roleSymbols);
      return;
    }

    addRoles(roleSymbols);

    getCurrentTeamStatus();

    findBestRoleAssignment(roleSymbols);

    takeNewestRoleAssignment(roleSymbols);
  }

  if (roleSymbols.role != lastRole)
    roleSymbols.lastRole = lastRole;

  ASSERT(roleSymbols.role < BehaviorData::numOfRoleAssignments);

  declareDebugDrawing(roleSymbols);
}

/**
 * \brief Declares a drawing visualizing the role distribution in the simulator.
 * 
 * Each robots player number will appear above its head with the color representing the role.
 * 
 * \param roleSymbols A copy of the current RoleSymbols representation.
 */
void RoleDynamicProvider::declareDebugDrawing(RoleSymbols& roleSymbols)
{
  DEBUG_DRAWING3D("representation:RoleSymbols", "robot")
  {
    // default color: keeper
    ColorRGBA digitColor = ColorRGBA::blue;
    switch (roleSymbols.role)
    {
    case BehaviorData::ballchaser:
      digitColor = ColorRGBA::red;
      break;
    case BehaviorData::defenderLeft:
      digitColor = ColorRGBA::green;
      break;
    case BehaviorData::defenderRight:
      digitColor = ColorRGBA::green;
      break;
    case BehaviorData::defenderSingle:
      digitColor = ColorRGBA::yellow;
      break;
    case BehaviorData::center:
      digitColor = ColorRGBA::white;
      break;
    case BehaviorData::backupBallchaser:
      digitColor = ColorRGBA::magenta;
      break;
    case BehaviorData::receiver:
      digitColor = ColorRGBA::orange;
      break;
    case BehaviorData::replacementKeeper:
      digitColor = ColorRGBA::cyan;
      break;
    default:
      break;
    }
    int pNumber = theRobotInfo.number;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    DRAWDIGIT3D("representation:RoleSymbols", pNumber, Vector3f(centerDigit, 0.f, 600), 100, 8, digitColor);
    Pose3f origin(0, 0, 370);
    origin.rotateY(90_deg);
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
  bool iAmBallChaser = theRobotInfo.number == theBallChaserDecision.playerNumberToBall;
  bool iAmKeeper = theRobotInfo.number == 1;
  if (!iAmBallChaser && !iAmKeeper && theRobotInfo.penalty == PENALTY_NONE)
  {
    playerNumbers.push_back(theRobotInfo.number);
    passiveRolePositions[theRobotInfo.number].clear();
    addMyPassiveRolePositions();
    robotPoses[theRobotInfo.number] = theRobotPoseAfterPreview;
  }
  // get information for teammates
  for (auto& mate : theTeammateData.teammates)
  {
    bool mateIsBallChaser = mate.number == theBallChaserDecision.playerNumberToBall;
    bool mateIsKeeper = mate.number == 1;
    if (!mateIsBallChaser && !mateIsKeeper && mate.status >= Teammate::Status::ACTIVE)
    {
      playerNumbers.push_back(mate.number);
      passiveRolePositions[mate.number].clear();
      for (const auto& prp : mate.behaviorData.passiveRolePositions)
        passiveRolePositions[mate.number].emplace_back(BehaviorData::PassiveRolePosition(prp.role, prp.position));
      robotPoses[mate.number] = mate.pose;
    }
  }
}

void RoleDynamicProvider::setStaticAssignment(RoleSymbols& roleSymbols)
{
  float ballDistance = theBallSymbols.ballPositionRelative.norm();
  bool teammateNearBall = false;
  for (const auto& robot : theRobotMap.robots)
  {
    if (robot.robotType == RobotEstimate::RobotType::teammateRobot && (robot.pose.translation - theBallSymbols.ballPositionField).norm() < 750.f)
      teammateNearBall = true;
  }
  float maxBallDistance = (theRobotInfo.number == 5) ? 3000.f : 2000.f;
  if (theRobotInfo.number == 1)
    roleSymbols.role = BehaviorData::RoleAssignment::keeper;
  else if ((theGameInfo.state == STATE_PLAYING && !teammateNearBall && ballDistance < ((roleSymbols.role == BehaviorData::RoleAssignment::ballchaser) ? (maxBallDistance + 500.f) : maxBallDistance))
      || (theGameInfo.state != STATE_PLAYING && theRobotInfo.number == 4))
    roleSymbols.role = BehaviorData::RoleAssignment::ballchaser;
  else if (theRobotInfo.number == 2)
    roleSymbols.role = BehaviorData::RoleAssignment::defenderLeft;
  else if (theRobotInfo.number == 3)
    roleSymbols.role = BehaviorData::RoleAssignment::defenderRight;
  else if (theRobotInfo.number == 4)
    roleSymbols.role = BehaviorData::RoleAssignment::center;
  else
    roleSymbols.role = BehaviorData::RoleAssignment::receiver;
}

void RoleDynamicProvider::addRoles(RoleSymbols& roleSymbols)
{
  roles.clear();
  for (const auto& role : theRoleSelection.selectedRoles)
  {
    // keeper and ballchaser are only roles not dynamically assigned
    if (role == BehaviorData::keeper)
      roleSymbols.roleSuggestions[1] = role;
    else if (role == BehaviorData::ballchaser && theBallChaserDecision.playerNumberToBall != 1)
      roleSymbols.roleSuggestions[theBallChaserDecision.playerNumberToBall] = BehaviorData::ballchaser;
    else
      roles.push_back(role);
  }
}

void RoleDynamicProvider::findBestRoleAssignment(RoleSymbols& roleSymbols)
{
  int playerCount = (int)roles.size();
  if (playerCount < 1)
    return; // nothing to do; i.e. only keeper and ballchaser were assigned
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

    std::vector<float> currentWalkDistances; // walking distances for this assignment
    for (int p = 0; p < playerCount; p++)
      currentWalkDistances.push_back(0.f);
    for (int j = 0; j < playerCount; j++)
    {
      Vector2f optPosition = robotPoses[j].translation;
      getPositionForRoleAndPlayer(roles[j], playerNumbers[j], optPosition);
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

  if (theRobotInfo.number == theBallChaserDecision.playerNumberToBall)
  {
    DEBUG_DRAWING3D("representation:RoleAssignment", "field")
    {
      for (size_t i = 0; i < roles.size(); i++)
      {
        Vector2f rolePassivePosition = robotPoses[i].translation;
        const int playerNum = optAssignment[i];
        const float drawOffset = 50.f; //add an small offset for drawing, otherwise the yellow path lines will may be visible
        getPositionForRoleAndPlayer(roles[i], playerNum, rolePassivePosition);
        LINE3D("representation:RoleAssignment",
            robotPoses[playerNum].translation.x() + drawOffset,
            robotPoses[playerNum].translation.y() + drawOffset,
            1.f,
            rolePassivePosition.x() + drawOffset,
            rolePassivePosition.y() + drawOffset,
            1.f,
            2.f,
            ColorRGBA::red);
        Vector2f diff = Vector2f(rolePassivePosition.x() - robotPoses[playerNum].translation.x(), rolePassivePosition.y() - robotPoses[playerNum].translation.y()).normalized();
        Vector3f from = Vector3f(rolePassivePosition.x() + drawOffset - 50 * diff.x(), rolePassivePosition.y() + drawOffset - 50 * diff.y(), 2.f);
        Vector3f to = Vector3f(rolePassivePosition.x() + drawOffset + 50 * diff.x(), rolePassivePosition.y() + drawOffset + 50 * diff.y(), 1.f);
        CYLINDERARROW3D("representation:RoleAssignment", from, to, 2.f, 90.f, 25.f, ColorRGBA::red);
      }
    }
  }
}

void RoleDynamicProvider::addMyPassiveRolePositions()
{
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::backupBallchaser, theBackupBallchaser.optPosition.translation));
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::center, theCenter.optPosition.translation));
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::defenderLeft, theDefenderLeft.optPosition.translation));
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::defenderRight, theDefenderRight.optPosition.translation));
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::defenderSingle, theDefenderSingle.optPosition.translation));
  passiveRolePositions[theRobotInfo.number].emplace_back(BehaviorData::PassiveRolePosition(BehaviorData::RoleAssignment::receiver, theReceiver.optPosition.translation));
}

void RoleDynamicProvider::getPositionForRoleAndPlayer(const BehaviorData::RoleAssignment role, const int playerNumber, Vector2f& position)
{
  for (size_t i = 0; i < passiveRolePositions[playerNumber].size(); i++)
    if (passiveRolePositions[playerNumber][i].role == role)
    {
      position = passiveRolePositions[playerNumber][i].position.cast<float>();
      return;
    }
}

void RoleDynamicProvider::takeNewestRoleAssignment(RoleSymbols& roleSymbols)
{
  const Teammate* teammate = theTeammateData.getNewestEventMessage(TeamCommEvents::SendReason::newRolesAssigned);
  if (teammate)
  {
    const auto& suggestions = teammate->behaviorData.roleSuggestions;
    for (int playerNum = 0; playerNum < static_cast<int>(suggestions.size()); ++playerNum)
    {
      if (suggestions[playerNum] == BehaviorData::RoleAssignment::ballchaser)
        lastPlayerNumberToBall = playerNum;
    }
  }
  const bool isOrWasStriker = theBallChaserDecision.playerNumberToBall == theRobotInfo.number || lastPlayerNumberToBall == theRobotInfo.number;
  lastPlayerNumberToBall = theBallChaserDecision.playerNumberToBall;

  if (isOrWasStriker || !teammate)
  {
    roleSymbols.role = roleSymbols.roleSuggestions[theRobotInfo.number];
  }
  else
  {
    roleSymbols.role = teammate->behaviorData.roleSuggestions[theRobotInfo.number];
    roleSymbols.roleSuggestions = teammate->behaviorData.roleSuggestions;
  }
}

MAKE_MODULE(RoleDynamicProvider, behaviorControl)
