/**
 * \file JoystickDeviceParameters.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file contains parameters for a specific joystick device.
 * Which device config file has to be loaded is previously loaded from general 
 * joystick config file.
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/MotionControl/WalkRequest.h"


// TODO: put implementation in cpp file?!
// TODO: special file for SpecialActionButtonPair, os is okay to have the class here?

STREAMABLE(JoystickDeviceParameters,
  /**
    * \class SpecialActionButtonPair
    *
    * This class is a pair of a special action and a button id. This class is
    * used to define special actions which are triggered by joystick button events.
    */
  STREAMABLE(SpecialActionButtonPair,,

    /// Define a special actions which is triggered by the given \c button id.
    (SpecialActionRequest) specialAction,

    /// Define a button id which triggers the special actions.
    (int)(0) button,

    /// Define minimum duration of the special action.
    (unsigned)(0) duration
  );

  /**
    * \class StepRequestButtonPair
    *
    * Class for storing information about the CustomStep execution via the left control keys
    */
  STREAMABLE(StepRequestButtonPair,,

    /// Define a CustomStep
    ((WalkRequest) StepRequest)(none) stepRequest,

    /// TODO
    (int)(0) stateValueX,

    /// TODO
    (int)(0) stateValueY,

    /// TODO
    (unsigned)(0) duration,

    /// TODO
    (bool)(false) mirror
  );

  void loadConfigFile(const std::string& fileName)
  {
    // Attempt to load config file with given name.
    InMapFile stream(fileName);
    if (!stream.exists())
    {
      // Fall back to default device file if file with fileName does not exist.
      stream = InMapFile("joystick_Default.cfg");
    }
    // Fill *this with contents of config file.
    ASSERT(stream.exists());
    stream >> *this;
  }

  /// Checks whether the rotation axis is split into left and right axes.
  bool walkRotAxisSplit() const
  {
    return walkRotLeftAxis >= 0 && (unsigned int)walkRotLeftAxis <= axesCount &&
           walkRotRightAxis >= 0 && (unsigned int)walkRotRightAxis <= axesCount;
  }
  ,

  /// Absolute axis values below this threshold does not cause a motion.
  (float)(0.f) minAxisValue,

  /// The number of avaiable buttons on the joystick.
  (unsigned int)(0) buttonsCount,
  /// The number of avaiable axes on the joystick.
  (unsigned int)(0) axesCount,

  /// Define axis which controls the walking speed in x-direction.
  /// Negative values deactivate x-direction walking control.
  (int)(0) walkXAxis,

  /// Define a second axis which controls the walking speed in x-direction.
  /// The values of \c walkXAxis and \c additionalWalkXAxis are added, 
  /// but cut off at max speed.
  (int)(0) additionalWalkXAxis,

  /// Define axis which controls the walking speed in y-direction.
  /// Negative values deactivate y-direction walking control.
  (int)(0) walkYAxis,

  /// Define axis which controls the rotational walking speed.
  /// Negative values deactivate rotational walking control.
  (int)(0) walkRotAxis,

  /// Define two axes which controls together the rotational walking speed.
  /// If walkRotAxis is valid, this axis is ignored.
  (int)(0) walkRotLeftAxis,

  /// Define two axes which controls together the rotational walking speed.
  /// If walkRotAxis is valid, this axis is ignored.
  (int)(0) walkRotRightAxis,

  /// Define axis which controls the head pan.
  /// Negative values deactivate head pan control.
  (int)(0) headPanAxis,
  /// Define axis which controls the head tilt.
  /// Negative values deactivate head tilt control.
  (int)(0) headTiltAxis,

  /// Define button which triggers the robot to stand up (same as chest button).
  (int)(0) startButton,

  /// Define button which triggers the robot to stand up (same as chest button).
  (int)(0) backButton,

  /// Define button which triggers a kick with the left foot.
  (int)(0) leftKickButton,
  /// Define button which triggers a kick with the right foot.
  (int)(0) rightKickButton,

  /// Define a list of special actions and button ids. The button in each of the 
  /// \c SpecialActionButtonPair elements triggers the related special action.
  (std::vector<SpecialActionButtonPair>) specialActions,

  /// Define the axis which controls the execution of CustomSteps
  (int)(0) customStepXAxis,

  (int)(0) customStepYAxis,

  (std::vector<StepRequestButtonPair>) activeKicks
);
