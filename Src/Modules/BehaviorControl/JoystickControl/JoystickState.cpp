/**
 * \file JoystickState.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file implements a class for parsing basic Joystick controls events
 */

#include "JoystickState.h"

#include "Tools/Module/Logger.h"
#include <sstream>
#include <iostream>


// Implementation of methods for class JoystickState:

JoystickState::JoystickState() :
    pressedButtons(std::vector<bool>(0)),
    axes(std::vector<float>(0)) {}

JoystickState::JoystickState(unsigned int buttonsCount, unsigned int axesCount) :
    pressedButtons(std::vector<bool>(buttonsCount)),
    axes(std::vector<float>(axesCount))
{
  // Initialize empty arrays.
  for (unsigned int i = 0; i < pressedButtons.size(); i++)
    pressedButtons[i] = false;
  for (unsigned int i = 0; i < axes.size(); i++)
    axes[i] = 0.f;
}

bool JoystickState::pressedButtonAvailable()
{
  for (size_t b = 0; b < pressedButtons.size(); b++)
  {
    if (pressedButtons[b]) return true;
  }
  return false;
}

void JoystickState::deleteEvents(const JoystickControlParameters &parameters)
{
  // Reinitialize all buttons with false.
  for (size_t b = 0; b < pressedButtons.size(); b++)
    pressedButtons[b] = false;

  // Reinitialize all axes with 0.
  for (size_t a = 0; a < axes.size(); a++)
    axes[a] = 0.f;
  // Reinitialize split axes with -1.
  if (parameters.device.walkRotAxisSplit())
  {
    axes[parameters.device.walkRotLeftAxis] = -1.f;
    axes[parameters.device.walkRotRightAxis] = -1.f;
  }
}
