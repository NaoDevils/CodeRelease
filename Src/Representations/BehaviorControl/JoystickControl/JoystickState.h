/**
 * \file JoystickState.h
 * \author Arne Moos <arne.moos@tu-dortmund.de>
 * \co-author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file declares the representation for parsing the basic Joystick controls events
 */
#pragma once

#include <climits>
#include <vector>

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"

#include "JoystickControlParameters.h"

STREAMABLE(JoystickState,
  /**
   * Creates a new \c JoystickState object with no buttons and no axes.
   */
  JoystickState();

  /**
   * Creates a new \c JoystickState object with the given button / axes count.
   */
  JoystickState(unsigned int buttonsCount, unsigned int axesCount);

  bool wasButtonReleasedThisFrame(unsigned int buttonID) const;

  /**
   * Delete all pressed button events and resets all axis values to 0.
   */
  void deleteEvents(const JoystickDeviceParameters& device);

  ,
  (unsigned)(0) timestampLastButtonEvent,
  (std::vector<int>) buttons,
  (std::vector<unsigned>) releasedButtonTimestamp,

  (unsigned)(0) timestampLastAxesEvent,
  (std::vector<float>) axes // Stores the values of all axes (range [-1.f, 1.f]).
);
