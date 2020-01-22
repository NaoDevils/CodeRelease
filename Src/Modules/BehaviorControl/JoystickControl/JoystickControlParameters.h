/**
 * \file JoystickControlParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file contains parameters for JoystickControl.
 */
#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

#include "JoystickDeviceParameters.h"

class JoystickControlParameters : public Streamable
{

public:

  JoystickControlParameters()
  {
    // Load general parameters.
    InMapFile stream("joystickControl.cfg");
	  ASSERT(stream.exists());
	  stream >> *this;

    // Load device-specific parameters.
    device.loadConfigFile(deviceParametersFileName);
  }
  
  /// Name of the config file with device-specific parameters.
  /// \see JoystickDeviceParameters
  std::string deviceParametersFileName;

  /// Device-specific parameters loaded from config file with
  /// name \c deviceParametersFileName.
  JoystickDeviceParameters device;

  /// Maximum walking velocity in x direction.
  float maxVelocityX;

  /// Minimum walking velocity in x direction.
  float minVelocityX;

  /// Maximum walking velocity in y direction.
  float maxVelocityY;

  /// Maximum rotational walking velocity.
  float maxVelocityRot;

  /// Maximum (= -minimum) value of HeadAngleRequest pan.
  float maxHeadPan;

  /// Minimum value of HeadAngleRequest tilt.
  float minHeadTilt;

  /// Maximum value of HeadAngleRequest tilt.
  float maxHeadTilt;

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;

    STREAM(deviceParametersFileName);

    STREAM(maxVelocityX);
    STREAM(minVelocityX);
    STREAM(maxVelocityY);
    STREAM(maxVelocityRot);

    STREAM(maxHeadPan);
    STREAM(minHeadTilt);
    STREAM(maxHeadTilt);

    STREAM_REGISTER_FINISH;
  }
};