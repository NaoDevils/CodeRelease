/**
 * \file JoystickStateProvider.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file implements a module that creates MotionRequests from a joystick input.
 */

#include "JoystickStateProvider.h"

#include <sstream>
#include <iostream>

JoystickStateProvider::JoystickStateProvider()
{
  initialize();
  localJoystickState.deleteEvents(device);
}

void JoystickStateProvider::update(JoystickState& theJoystickState)
{
  theJoystickState = localJoystickState;
}

void JoystickStateProvider::update(JoystickControlParameters& theJoystickControlParameters)
{
  theJoystickControlParameters = parameters;
}

void JoystickStateProvider::update(JoystickDeviceParameters& theJoystickDeviceParameters)
{
  theJoystickDeviceParameters = device;
}

void JoystickStateProvider::execute(tf::Subflow&)
{
  MODIFY("module:JoystickStateProvider:parameters", parameters);
  MODIFY("module:JoystickStateProvider:deviceParameters", device);

  // Read all pending joystick events.
  parseJoystickEvents();
}

void JoystickStateProvider::initialize()
{
  // Load general parameters.
  InMapFile stream("joystickController.cfg");
  ASSERT(stream.exists());
  stream >> parameters;

  // Load device-specific parameters.
  device.loadConfigFile(parameters.deviceParametersFileName);

  // Check connection to a joystick
  joystick.connect();

  // TODO: may play sound for connection?
  if (!joystick.isFound())
  {
    OUTPUT(idText, text, "Failed to open joystick.");
  }

  // Create joystick state object which stores state of buttons and axes.
  // Initializes all buttons with false and all axes with 0.
  localJoystickState = JoystickState(device.buttonsCount, device.axesCount);

  // Initialize split axes with -1.
  if (device.walkRotAxisSplit())
  {
    localJoystickState.axes[device.walkRotLeftAxis] = -1.f;
    localJoystickState.axes[device.walkRotRightAxis] = -1.f;
  }
}

void JoystickStateProvider::parseJoystickEvents()
{
#ifdef LINUX
  // Check if joystick is connected.
  if (!joystick.isFound())
  {
    // Try to reconnect.
    joystick.connect();
    if (!joystick.isFound())
    {
      // Connection failed.
      localJoystickState.deleteEvents(device);
      return;
    }
  }

  // Receive new joystick events.
  JoystickEvent event;
  while (joystick.sample(&event))
  {
    if (event.isButton())
    {
      // Received a button event (pressed/released).
#ifdef TEXT_OUTPUT
      // Output event description.
      std::stringstream ss;
      ss << "Joystick: Button " << (int)event.number << " is " << (event.value == 0 ? "up" : "down");
      OUTPUT(idText, text, ss.str());
      std::cout << ss.str() << std::endl;
#endif

      if (event.value == 0 && localJoystickState.buttons[event.number] == 1)
      {
        localJoystickState.timestampLastButtonEvent = theFrameInfo.time;
        localJoystickState.releasedButtonTimestamp[event.number] = theFrameInfo.time;
      }

      // Button <event.number> was pressed. Save this in joystick localJoystickState.
      localJoystickState.buttons[event.number] = event.value;
    }
    else
    {
      localJoystickState.timestampLastAxesEvent = theFrameInfo.time;
      // Received an axis event.
#ifdef TEXT_OUTPUT
      // Output event description.
      std::stringstream ss;
      ss << "Joystick: Axis " << (int)event.number << " is at position " << event.value;
      OUTPUT(idText, text, ss.str());
      std::cout << ss.str() << std::endl;
#endif

      // Axes <event.number> was moved. Save axis value mapped to range [-1,1] in joystick localJoystickState.
      localJoystickState.axes[event.number] = (float)event.value / (float)SHRT_MAX;

      // Cut off the axis value at about -1 if the axis is combined with another one.
      if (device.walkRotAxisSplit() && (event.number == device.walkRotLeftAxis || event.number == device.walkRotRightAxis))
      {
        if (localJoystickState.axes[event.number] < (-1.f + device.minAxisValue))
        {
          localJoystickState.axes[event.number] = -1.f;
        }
      }
      // Cut off the axis value at the threshold minAxisValue.
      else if (fabs(localJoystickState.axes[event.number]) < device.minAxisValue)
      {
        localJoystickState.axes[event.number] = 0.f;
      }

      if (event.number == device.customStepXAxis || event.number == device.customStepYAxis)
      {
        localJoystickState.axes[event.number] = (float)event.value / (float)SHRT_MAX;
      }
    }
  }
#endif
}

MAKE_MODULE(JoystickStateProvider, behaviorControl)
