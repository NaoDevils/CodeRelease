/**
 * \file JoystickState.cpp
 * \author Arne Moos <arne.moos@tu-dortmund.de>
 * \co-author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file implements the methods of the representation for parsing the basic Joystick controls events
 */

#include "Representations/Infrastructure/FrameInfo.h"
#include "JoystickState.h"

#include <sstream>
#include <iostream>


// Implementation of methods for class JoystickState:

JoystickState::JoystickState()
    : timestampLastButtonEvent(0), buttons(std::vector<int>(0)), releasedButtonTimestamp(std::vector<unsigned>(0)), timestampLastAxesEvent(0), axes(std::vector<float>(0))
{
}

JoystickState::JoystickState(unsigned int buttonsCount, unsigned int axesCount)
    : timestampLastButtonEvent(0), buttons(std::vector<int>(buttonsCount)), releasedButtonTimestamp(std::vector<unsigned>(buttonsCount)), timestampLastAxesEvent(0),
      axes(std::vector<float>(axesCount))
{
  // Initialize empty arrays.
  for (size_t b = 0; b < buttons.size(); b++)
  {
    buttons[b] = 0;
    releasedButtonTimestamp[b] = 0;
  }

  // Reinitialize all axes with 0.
  for (size_t a = 0; a < axes.size(); a++)
    axes[a] = 0.f;
}

bool JoystickState::wasButtonReleasedThisFrame(unsigned int buttonID) const
{
  const FrameInfo& theFrameInfo = Blackboard::get<FrameInfo>();
  if (std::abs(theFrameInfo.getTimeSince(releasedButtonTimestamp[buttonID])) < (theFrameInfo.cycleTime * 1000))
    return true;
  return false;
}

void JoystickState::deleteEvents(const JoystickDeviceParameters& device)
{
  // Reinitialize all buttons with false.
  for (size_t b = 0; b < buttons.size(); b++)
  {
    buttons[b] = 0;
    releasedButtonTimestamp[b] = 0;
  }

  // Reinitialize all axes with 0.
  for (size_t a = 0; a < axes.size(); a++)
    axes[a] = 0.f;

  // Reinitialize split axes with -1.
  if (device.walkRotAxisSplit())
  {
    axes[device.walkRotLeftAxis] = -1.f;
    axes[device.walkRotRightAxis] = -1.f;
  }
}
