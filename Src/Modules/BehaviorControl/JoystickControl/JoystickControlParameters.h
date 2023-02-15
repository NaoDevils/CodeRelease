/**
 * \file JoystickControlParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file contains parameters for JoystickControl.
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

#include "JoystickDeviceParameters.h"

STREAMABLE(JoystickControlParameters,
  JoystickControlParameters()
  {
    // Load general parameters.
    InMapFile stream("joystickControl.cfg");
    ASSERT(stream.exists());
    stream >> *this;

    // Load device-specific parameters.
    device.loadConfigFile(deviceParametersFileName);
  }

  /// Device-specific parameters loaded from config file with
  /// name \c deviceParametersFileName.
  JoystickDeviceParameters device;
  ,
  
  /// Name of the config file with device-specific parameters.
  /// \see JoystickDeviceParameters
  (std::string)("") deviceParametersFileName,

  /// Maximum walking velocity in x direction.
  (float)(0.f) maxVelocityX,

  /// Minimum walking velocity in x direction.
  (float)(0.f) minVelocityX,

  /// Maximum walking velocity in y direction.
  (float)(0.f) maxVelocityY,

  /// Maximum rotational walking velocity.
  (float)(0.f) maxVelocityRot,

  /// Maximum (= -minimum) value of HeadAngleRequest pan.
  (float)(0.f) maxHeadPan,

  /// Minimum value of HeadAngleRequest tilt.
  (float)(0.f) minHeadTilt,

  /// Maximum value of HeadAngleRequest tilt.
  (float)(0.f) maxHeadTilt
);
