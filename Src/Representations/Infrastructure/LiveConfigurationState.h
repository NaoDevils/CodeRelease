/**
* \file LiveConfigurationState.h

* The file declares a class containing information needed to configure the robot on the field.
*/

#pragma once
#include "Representations/BehaviorControl/BehaviorData.h"

/**
 * \class LiveConfigurationState
 * 
 * Declaration of the LiveConfigurationState representation.
 * 
 * In order to provide easier testing on the field the robots should be configurable without the
 * use of the simulator and the need to of a redeploy after configuration changes. Since 
 * multiple settings could be subject to change the LiveConfigurationState centralizes information
 * about which setting is currently being changed. A press of the middle head touch button 
 * changes the configuration state. Other modules can require the LiveConfigurationState in order
 * to react to button presses given that the correct configuration state is active.
 * 
 * Example:
 * A special role provider can be used in tests with a single robot that uses a fixed role and 
 * changes that role on a press of the rear/front head button, but only if the LiveConfigurationState
 * is set to role configuration.
 */

STREAMABLE(LiveConfigurationState,
{
  ENUM(ConfigurationState,
  {,
    firstState,
    noConfiguration = firstState,
    lastState,
    singleRoleConfiguration = lastState,
  });
  typedef std::vector<ConfigurationState> ConfigurationStateVector;
  ,
  (bool)(false) liveConfigurationActive,
  (ConfigurationState)(noConfiguration) currentConfigurationState,
  // store information about the states of the head touch buttons
  (bool)(false) headFrontPressed,
  (bool)(false) headMiddlePressed,
  (bool)(false) headRearPressed,
  (bool)(false) headFrontPressedThisFrame,
  (bool)(false) headMiddlePressedThisFrame,
  (bool)(false) headRearPressedThisFrame,
});