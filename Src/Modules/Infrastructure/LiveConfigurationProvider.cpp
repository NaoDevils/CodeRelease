/**
 * @file LiveConfigurationProvider.cpp
 *
 * Implementation of class LiveConfigurationProvider.
 */

#include "LiveConfigurationProvider.h"

void LiveConfigurationProvider::update(LiveConfigurationState& configState)
{
  configState.liveConfigurationActive = enableLiveConfiguration;
  if (!enableLiveConfiguration)
  {
    return;
  }
  updateKeyStates(configState);
  updateConfigurationState(configState);
}

/**
 * \brief Updates the member variables that keep track of the head button's states.
 * 
 * The robot memorizes if a button is being pressed in this frame in the *Pressed variables.
 * It uses those variables to determine if a button press started in this frame by checking if 
 * the button was not pressed last frame (*Pressed equals false) but is pressed now. It saves the
 * result in the *PressedThisFrame variables.
 * Uses information that was stored in theLiveConfigurationState before to determine state 
 * changes of button presses.
 * 
 * \param configState The LiveConfigurationState to be filled this frame.
 */
void LiveConfigurationProvider::updateKeyStates(LiveConfigurationState& configState)
{
  // store information about key presses
  configState.headFrontPressed = theKeyStates.pressed[KeyStates::Key::headFront];
  configState.headMiddlePressed = theKeyStates.pressed[KeyStates::Key::headMiddle];
  configState.headRearPressed = theKeyStates.pressed[KeyStates::Key::headRear];
  // the press started this frame if button was not pressed before but is pressed now
  configState.headFrontPressedThisFrame = !headFrontPressedLastFrame && configState.headFrontPressed;
  configState.headMiddlePressedThisFrame = !headMiddlePressedLastFrame && configState.headMiddlePressed;
  configState.headRearPressedThisFrame = !headRearPressedLastFram && configState.headRearPressed;
  // memorize if button was pressed for next frame
  headFrontPressedLastFrame = configState.headFrontPressed;
  headMiddlePressedLastFrame = configState.headMiddlePressed;
  headRearPressedLastFram = configState.headRearPressed;
}

/**
 * \brief Changes the current configuration state when the middle head button is pressed.
 */
void LiveConfigurationProvider::updateConfigurationState(LiveConfigurationState& configState)
{
  if (configState.headMiddlePressedThisFrame)
  {
    currentStateIndex++;
    // rollover when end of availableConfigStates is passed
    currentStateIndex = (currentStateIndex >= (int)availableConfigStates.size()) ? 0 : currentStateIndex;
    configState.currentConfigurationState = availableConfigStates[currentStateIndex];
  }
}

MAKE_MODULE(LiveConfigurationProvider, cognitionInfrastructure)