/**
* @file RoleTestProvider.cpp
*
* Implementation of class RoleTestProvider.
*
*/

#include "RoleTestProvider.h"
#include "Tools/Debugging/Annotation.h"
#include <algorithm>

void RoleTestProvider::update(RoleSymbols& roleSymbols)
{
  std::vector<int> playerNumbers; // which players are present?
  // get player numbers and current position of all active robots in the team
  getCurrentTeamStatus(playerNumbers);

  roleSymbols.roleSuggestions.clear();
  for (int i = 0; i <= MAX_NUM_PLAYERS; i++)
    roleSymbols.roleSuggestions.push_back(BehaviorData::noRole);
  for (size_t i = 0; i < playerNumbers.size(); i++)
    roleSymbols.roleSuggestions[playerNumbers[i]] = rolesToTest[i];
  roleSymbols.lastRole = roleSymbols.role;
  roleSymbols.role = roleSymbols.roleSuggestions[theRobotInfo.number];
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
 * \param playerNumbers The vector that will be filled with the player numbers of active robots.
 *                        Should be empty beforehand.
 * \param robotPoses The vector that will be filled with the players current positions. Should be
 *                    filled with empty poses equal to the number of players + 1.
 */
void RoleTestProvider::getCurrentTeamStatus(std::vector<int>& playerNumbers)
{
  // get information for myself
  playerNumbers.push_back(theRobotInfo.number);

  // get information for teammates
  for (auto& mate : theTeammateData.teammates)
  {
    playerNumbers.push_back(mate.number);
  }
}

MAKE_MODULE(RoleTestProvider, behaviorControl)