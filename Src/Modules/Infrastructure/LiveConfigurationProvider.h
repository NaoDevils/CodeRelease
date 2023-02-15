/**
 * @file LiveConfigurationProvider.h
 *
 * Declaration of class LiveConfigurationProvider
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/LiveConfigurationState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(LiveConfigurationProvider,
  REQUIRES(KeyStates),
  PROVIDES(LiveConfigurationState),
  LOADS_PARAMETERS(,
    (bool)(false) enableLiveConfiguration,
    ((LiveConfigurationState) ConfigurationStateVector) availableConfigStates
  )
);

/**
 * @class LiveConfigurationProvider
 */
class LiveConfigurationProvider : public LiveConfigurationProviderBase
{
public:
  /** Constructor */
  LiveConfigurationProvider()
  {
    currentStateIndex = 0;
    headFrontPressedLastFrame = false;
    headMiddlePressedLastFrame = false;
    headRearPressedLastFram = false;
  }

private:
  /** Fill the representation for the current frame. */
  void update(LiveConfigurationState& configState);
  /** Update the current states of the head touch buttons. */
  void updateKeyStates(LiveConfigurationState& configState);
  /** Switches the configuration state if necessary. */
  void updateConfigurationState(LiveConfigurationState& configState);

  // vvv--- member variables ---vvv
  // the index in availableConfigStates of the currently active configuration state
  int currentStateIndex;
  // memorize state of head buttons for next frame
  bool headFrontPressedLastFrame, headMiddlePressedLastFrame, headRearPressedLastFram;
};
