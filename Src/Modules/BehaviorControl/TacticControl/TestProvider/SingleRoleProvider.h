/**
 * @file SingleRoleProvider.h
 *
 * Declaration of class SingleRoleProvider.h
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/LiveConfigurationState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols.h"


MODULE(SingleRoleProvider,
  REQUIRES(KeyStates),
  REQUIRES(LiveConfigurationState),
  REQUIRES(RobotInfo),
  PROVIDES(RoleSymbols)
);

/**
 * @class SingleRoleProvider
 * 
 * RoleProvider for tests of specific roles with only one robot.
 * 
 * The robot chooses one role at the start and does not change it dynamically. The head buttons
 * of the robot can be used to change the role of the robot allowing for simpler tests of 
 * certain roles.
 * 
 * This provider uses the LiveConfigurationState (for more information check the LiveConfigurationState
 * documentation.) Role changes are only possible if live configuration is enabled and SingleRoleSelection
 * is one of the possible configuration states (both set in config of LiveConfigurationProvider). 
 * When SingleRoleSelection is the currently active live configuration state presses of the front and rear 
 * head buttons rotate through the different roles. Which role is currently selected is 
 * indicated by the right eye leds.
 */
class SingleRoleProvider : public SingleRoleProviderBase
{
public:
  /** Constructor */
  SingleRoleProvider()
  {
    // start as keeper
    currentRoleNumber = 1;
    currentRole = BehaviorData::RoleAssignment(currentRoleNumber);
  }

private:
  /** Fills the role symbols for the current frame. */
  void update(RoleSymbols& roleSymbols);
  /** Switches to the previous/next role as the current role. */
  void shiftRole(bool forward);
  /** Executes the role shift if all conditions are met. */
  void checkForRoleShift();
  /** Get a role suggestion vector for the role symbols. */
  BehaviorData::RobotRoleAssignmentVector getRoleSuggestions();


  // vvv--- member variables ---vvv
  // keep track of the current role
  int currentRoleNumber;
  BehaviorData::RoleAssignment currentRole;
};
