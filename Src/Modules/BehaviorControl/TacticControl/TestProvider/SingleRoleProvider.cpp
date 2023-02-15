/**
 * @file SingleRoleProvider.cpp
 *
 * Implementation of class SingleRoleProvider.
 */

#include "SingleRoleProvider.h"
#include "Tools/Debugging/Annotation.h"

// #include <algorithm>

void SingleRoleProvider::update(RoleSymbols& roleSymbols)
{
  checkForRoleShift();
  roleSymbols.lastRole = roleSymbols.role;
  roleSymbols.role = currentRole;
  roleSymbols.roleSuggestions = getRoleSuggestions();
}

/**
 * \brief Selects the previous/next role as the current role.
 * 
 * \param forward If true the next role is selected, otherwise the previous will be chosen.
 */
void SingleRoleProvider::shiftRole(bool forward)
{
  int shiftOffset = forward ? 1 : -1;
  int nextRoleNumber = currentRoleNumber + shiftOffset;
  // skip the undefined role
  if (BehaviorData::RoleAssignment(nextRoleNumber) == BehaviorData::RoleAssignment::noRole)
    nextRoleNumber += shiftOffset;
  if (nextRoleNumber < 0)
    nextRoleNumber = (int)BehaviorData::RoleAssignment::numOfRoleAssignments - 1;
  if (nextRoleNumber >= (int)BehaviorData::RoleAssignment::numOfRoleAssignments)
    nextRoleNumber = 1;
  currentRoleNumber = nextRoleNumber;
  currentRole = BehaviorData::RoleAssignment(currentRoleNumber);
}

/**
 * \brief Executes a role shift if all conditions are met.
 * 
 * A role shift occurs if the LiveConfigurationState is set to singleRoleConfiguration and
 * either the rear or front button was pressed this frame. Which of the two buttons was pressed 
 * determines the direction of the shift.
 */
void SingleRoleProvider::checkForRoleShift()
{
  if (theLiveConfigurationState.liveConfigurationActive && theLiveConfigurationState.currentConfigurationState == LiveConfigurationState::ConfigurationState::singleRoleConfiguration)
  {
    if (theLiveConfigurationState.headFrontPressedThisFrame)
      shiftRole(true);
    // else => prioritize front button on simultaneous press
    else if (theLiveConfigurationState.headRearPressedThisFrame)
      shiftRole(false);
  }
}

/**
 * \brief Returns a role vector suggesting the current role for this robot and undefined for 
 *         everybody else.
 */
BehaviorData::RobotRoleAssignmentVector SingleRoleProvider::getRoleSuggestions()
{
  BehaviorData::RobotRoleAssignmentVector roleSuggestions(MAX_NUM_PLAYERS, BehaviorData::noRole);
  roleSuggestions[theRobotInfo.number] = currentRole;
  return roleSuggestions;
}

MAKE_MODULE(SingleRoleProvider, behaviorControl)