/**
 * \file JoystickStateProvider.h
 * \author Arne Moos <arne.moos@tu-dortmund.de>
 *
 * This file declares a module that fills the JoystickState from a joystick input.
 */
#pragma once

#include <vector>

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickState.h"
#include "Representations/BehaviorControl/JoystickControl/JoystickControlParameters.h"

#include "Modules/BehaviorControl/JoystickControl/JoystickLib.h"

// Send debug messages for received joystick events.
//#define TEXT_OUTPUT

MODULE(JoystickStateProvider,
  REQUIRES(FrameInfo),
  PROVIDES(JoystickState),
  PROVIDES(JoystickControlParameters),
  PROVIDES(JoystickDeviceParameters),
  HAS_PREEXECUTION
);


class JoystickStateProvider : public JoystickStateProviderBase
{
public:
  JoystickStateProvider();

  void update(JoystickState& theJoystickState);
  void update(JoystickControlParameters& theJoystickControlParameters);
  void update(JoystickDeviceParameters& theJoystickDeviceParameters);

private:
  void execute(tf::Subflow&);

  void initialize();

  void parseJoystickEvents();

  JoystickLib joystick;
  JoystickDeviceParameters device;

  JoystickControlParameters parameters;

  JoystickState localJoystickState;
};
