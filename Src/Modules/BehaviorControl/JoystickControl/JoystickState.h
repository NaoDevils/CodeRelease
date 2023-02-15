/**
 * \file JoystickState.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file declares a class for parsing basic Joystick controls events
 */
#pragma once

#include <climits>
#include <vector>

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"

#include "JoystickControlParameters.h"


// Send debug messages for received joystick events.
//#define TEXT_OUTPUT

/**
 * \class JoystickState
 * 
 * This class stores pressed buttons until they are processed and 
 * the current state of all axes.
 */
class JoystickState
{
public:
  /**
   * Creates a new \c JoystickState object with no buttons and no axes.
   */
  JoystickState();

  /**
   * Creates a new \c JoystickState object with the given button / axes count.
   */
  JoystickState(unsigned int buttonsCount, unsigned int axesCount);

  /**
   * Checks whether at least one button was recently pressed (\c pressedButtons
   * contains at least one entry with value \c true).
   */
  bool pressedButtonAvailable();

  /**
   * Delete all pressed button events and resets all axis values to 0.
   */
  void deleteEvents(const JoystickControlParameters& parameters);

public:
  /// Contains a bool value for each button.
  /// pressedButtons(i) = true: Event 'Button i pressed' was not processed yet.
  std::vector<bool> pressedButtons;

  /// Stores the values of all axes (range [-1.f, 1.f]).
  std::vector<float> axes;
};
