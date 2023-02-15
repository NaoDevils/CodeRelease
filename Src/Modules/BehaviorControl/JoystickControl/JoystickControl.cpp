/**
 * \file JoystickControl.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * This file implements a module that creates MotionRequests from a joystick input.
 */

#include "JoystickControl.h"

#include <sstream>
#include <iostream>

MAKE_MODULE(JoystickControl, behaviorControl)


JoystickControl::JoystickControl() : lastExecuteTimeStamp(0), m_inCustomStepTest(false)
{
  initialize();
}

JoystickControl::~JoystickControl() {}

void JoystickControl::update(ArmContact& armContact)
{
  armContact.armContactStateLeft = ArmContact::None;
  armContact.armContactStateRight = ArmContact::None;
  armContact.timeStampLeft = 0;
  armContact.timeStampRight = 0;
}

void JoystickControl::update(HeadControlRequest& headControlRequest)
{
  execute();

  headControlRequest = localHeadControlRequest;
}

void JoystickControl::update(MotionRequest& motionRequest)
{
  execute();

  motionRequest = localMotionRequest;
}

void JoystickControl::initialize()
{
  // Check connection to a joystick
  joystick.connect();

  // TODO: may play sound for connection?
  if (!joystick.isFound())
  {
    OUTPUT(idText, text, "Failed to open joystick.");
  }

  // Create joystick state object which stores state of buttons and axes.
  // Initializes all buttons with false and all axes with 0.
  state = JoystickState(parameters.device.buttonsCount, parameters.device.axesCount);

  // Initialize split axes with -1.
  if (parameters.device.walkRotAxisSplit())
  {
    state.axes[parameters.device.walkRotLeftAxis] = -1.f;
    state.axes[parameters.device.walkRotRightAxis] = -1.f;
  }

  // Robot starts with centered head.
  localHeadControlRequest.controlType = HeadControlRequest::direct;
  localHeadControlRequest.pan = 0.f;
  localHeadControlRequest.tilt = 0.f;

  // Robot starts playing dead.
  m_standing = false;
  playDead();
  localMotionRequest.walkRequest.requestType = WalkRequest::speed;
  localMotionRequest.walkRequest.request.translation << 0.f, 0.f;
  localMotionRequest.walkRequest.request.rotation = 0.f;
  localMotionRequest.kickRequest.kickMotionType = KickRequest::none;

  // No action running.
  m_actionRunning = false;
  m_actionEndTime = 0;
}

void JoystickControl::execute()
{
  // Run this method only once per frame.
  if (lastExecuteTimeStamp == theFrameInfo.time)
    return;

  lastExecuteTimeStamp = theFrameInfo.time;

  MODIFY("module:JoystickControl:parameters", parameters);
  MODIFY("module:JoystickControl:deviceParameters", parameters.device);

  // Read all pending joystick events.
  parseJoystickEvents();

  // Process joystick events.
  generateHeadControlRequest();
  generateMotionRequest();
}

void JoystickControl::parseJoystickEvents()
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
      state.deleteEvents(parameters);
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

      // Don't care about button release.
      if (event.value == 0)
        continue;

      // Button <event.number> was pressed. Save this in joystick state.
      state.pressedButtons[event.number] = true;
    }
    else
    {
      // Received an axis event.
#ifdef TEXT_OUTPUT
      // Output event description.
      std::stringstream ss;
      ss << "Joystick: Axis " << (int)event.number << " is at position " << event.value;
      OUTPUT(idText, text, ss.str());
      std::cout << ss.str() << std::endl;
#endif

      // Axes <event.number> was moved. Save axis value mapped to range [-1,1] in joystick state.
      state.axes[event.number] = (float)event.value / (float)SHRT_MAX;

      // Cut off the axis value at about -1 if the axis is combined with another one.
      if (parameters.device.walkRotAxisSplit() && (event.number == parameters.device.walkRotLeftAxis || event.number == parameters.device.walkRotRightAxis))
      {
        if (state.axes[event.number] < (-1.f + parameters.device.minAxisValue))
        {
          state.axes[event.number] = -1.f;
        }
      }
      // Cut off the axis value at the threshold minAxisValue.
      else if (fabs(state.axes[event.number]) < parameters.device.minAxisValue)
      {
        state.axes[event.number] = 0.f;
      }

      if (event.number == parameters.device.customStepXAxis || event.number == parameters.device.customStepYAxis)
      {
        state.axes[event.number] = (float)event.value / (float)SHRT_MAX;
      }
    }
  }
#endif
}

void JoystickControl::processChestButton()
{
  // Was the chest button pressed?
  if (theKeyStates.pressed[KeyStates::chest]
      || (parameters.device.startButton >= 0 && (unsigned int)parameters.device.startButton <= parameters.device.buttonsCount && state.pressedButtons[parameters.device.startButton]))
  {
    // Ignore all incoming button events when the chest button was pressed.
    state.deleteEvents(parameters);

    // Change standing state.
    if (standing())
    {
      sitDown();

      // Reset joystick connection.
      state.deleteEvents(parameters);

      // TODO: check when to execute
      joystick.connect();
    }
    else
    {
      stand();
    }

    m_standing = !m_standing;
  }
}

void JoystickControl::generateHeadControlRequest()
{
  localHeadControlRequest.controlType = HeadControlRequest::direct;

  // Set pan.
  if (parameters.device.headPanAxis >= 0 && (unsigned int)parameters.device.headPanAxis <= parameters.device.axesCount)
    localHeadControlRequest.pan = -state.axes[parameters.device.headPanAxis] * parameters.maxHeadPan;
  else
    localHeadControlRequest.pan = 0.f;

  // Set tilt.
  if (parameters.device.headTiltAxis >= 0 && (unsigned int)parameters.device.headTiltAxis <= parameters.device.axesCount)
  {
    float maxTilt = std::fmaxf(fabsf(parameters.maxHeadTilt), fabsf(parameters.minHeadTilt));
    localHeadControlRequest.tilt = -state.axes[parameters.device.headTiltAxis] * maxTilt;
    if (localHeadControlRequest.tilt < parameters.minHeadTilt)
      localHeadControlRequest.tilt = parameters.minHeadTilt;
    if (localHeadControlRequest.tilt > parameters.maxHeadTilt)
      localHeadControlRequest.tilt = parameters.maxHeadTilt;
  }
  else
    localHeadControlRequest.tilt = 0.f;
}

void JoystickControl::generateMotionRequest()
{
  // Keep last motion request while the action is still running.
  if (actionRunning())
  {
    // Ignore all incoming button events while an action is executed.
    state.deleteEvents(parameters);
    return;
  }

  processChestButton();

  // Don't allow any other action while the robot does not stand.
  if (!standing())
  {
    if (!actionRunning())
      playDead();
    return;
  }

  // TODO put in extra method

  // Walk at speed.
  localMotionRequest.motion = MotionRequest::walk;
  localMotionRequest.walkRequest.requestType = WalkRequest::speed;

  if (parameters.device.backButton >= 0 && (unsigned int)parameters.device.backButton <= parameters.device.buttonsCount && state.pressedButtons[parameters.device.backButton])
  {
    m_inCustomStepTest = !m_inCustomStepTest;
  }

  if (m_inCustomStepTest)
  {
    localMotionRequest.walkRequest.request.translation[0] = 0.5f;
  }

  // Evaluate button press if available.
  if (state.pressedButtonAvailable())
  {
    generateActionRequest();
  }
  else
  {
    generateWalkRequest();
    generateCustomStep();
  }
}

void JoystickControl::generateActionRequest()
{
  // Trigger left kick.
  if (parameters.device.leftKickButton >= 0 && static_cast<unsigned int>(parameters.device.leftKickButton) <= parameters.device.buttonsCount)
  {
    if (state.pressedButtons[parameters.device.leftKickButton])
    {
      kick(true);
      return;
    }
  }
  // Trigger right kick.
  if (parameters.device.rightKickButton >= 0 && static_cast<unsigned int>(parameters.device.rightKickButton) <= parameters.device.buttonsCount)
  {
    if (state.pressedButtons[parameters.device.rightKickButton])
    {
      kick(false);
      return;
    }
  }

  // Trigger special actions.
  for (size_t s = 0; s < parameters.device.specialActions.size(); s++)
  {
    if (parameters.device.specialActions[s].button >= 0 && static_cast<unsigned int>(parameters.device.specialActions[s].button) <= parameters.device.buttonsCount)
    {
      if (state.pressedButtons[parameters.device.specialActions[s].button])
      {
        specialAction(parameters.device.specialActions[s].specialAction, parameters.device.specialActions[s].duration);
        return;
      }
    }
  }

  // Unassigned button was pressed -> clear states!
  for (size_t b = 0; b < state.pressedButtons.size(); b++)
    state.pressedButtons[b] = false;
}

bool JoystickControl::standing()
{
  if (!m_standing)
    // The robot is sitting. Press chest button to stand up.
    return false;

  // TODO correct checking of FallDownState
  if (theFallDownState.state == FallDownState::State::onGround || theFallDownState.state == FallDownState::State::falling)
  {
    // Robot is falling or fallen.
    standUpFallen();
    return false;
  }
  return true;
}

bool JoystickControl::actionRunning()
{
  if (m_actionRunning && theFrameInfo.time > m_actionEndTime)
  {
    // Kick motion waits until leaving is possible.
    if (localMotionRequest.motion == MotionRequest::kick)
    {
      if (theKickEngineOutput.isLeavingPossible)
      {
        m_actionRunning = false;
      }
    }

    // Other motion leaves after time is out. This should not be reached, for
    // there is no other motion type which causes an JoystickControl action.
    else
    {
      m_actionRunning = false;
    }
  }
  return m_actionRunning;
}

void JoystickControl::generateWalkRequest()
{
  if (!m_inCustomStepTest)
  {
    // Walk at speed.
    localMotionRequest.motion = MotionRequest::walk;
    localMotionRequest.walkRequest.requestType = WalkRequest::speed;

    if (parameters.device.backButton >= 0 && (unsigned int)parameters.device.backButton <= parameters.device.buttonsCount && state.pressedButtons[parameters.device.backButton])
    {
      m_inCustomStepTest = !m_inCustomStepTest;
    }

    if (m_inCustomStepTest)
    {
      localMotionRequest.walkRequest.request.translation[0] = 10.f;
    }

    // Walking direction x.
    if (parameters.device.walkXAxis >= 0 && static_cast<unsigned int>(parameters.device.walkXAxis) <= parameters.device.axesCount)
      localMotionRequest.walkRequest.request.translation[0] = -state.axes[parameters.device.walkXAxis] * parameters.maxVelocityX;
    else
      localMotionRequest.walkRequest.request.translation[0] = 0.f;

    // Add value of additional walkXAxis.
    if (parameters.device.additionalWalkXAxis >= 0 && static_cast<unsigned int>(parameters.device.additionalWalkXAxis) <= parameters.device.axesCount)
    {
      localMotionRequest.walkRequest.request.translation[0] = localMotionRequest.walkRequest.request.translation.x()
          - state.axes[parameters.device.additionalWalkXAxis] * parameters.maxVelocityX;
    }
    if (localMotionRequest.walkRequest.request.translation.x() > 0.0 && localMotionRequest.walkRequest.request.translation.x() > parameters.maxVelocityX)
      localMotionRequest.walkRequest.request.translation[0] = parameters.maxVelocityX;
    if (localMotionRequest.walkRequest.request.translation.x() < 0.0 && localMotionRequest.walkRequest.request.translation.x() < parameters.minVelocityX)
      localMotionRequest.walkRequest.request.translation[0] = parameters.minVelocityX;

    // Walking direction y.
    if (parameters.device.walkYAxis >= 0 && static_cast<unsigned int>(parameters.device.walkYAxis) <= parameters.device.axesCount)
      localMotionRequest.walkRequest.request.translation[1] = -state.axes[parameters.device.walkYAxis] * parameters.maxVelocityY;
    else
      localMotionRequest.walkRequest.request.translation[1] = 0.f;

    // Walking rotation.
    if (parameters.device.walkRotAxis >= 0 && static_cast<unsigned int>(parameters.device.walkRotAxis) <= parameters.device.axesCount)
      localMotionRequest.walkRequest.request.rotation = -state.axes[parameters.device.walkRotAxis] * parameters.maxVelocityRot;
    else if (parameters.device.walkRotAxisSplit())
    {
      float left = (state.axes[parameters.device.walkRotLeftAxis] + 1.f) / 2.f; // range [0,1]
      float right = (state.axes[parameters.device.walkRotRightAxis] + 1.f) / 2.f; // range [0,1]
      localMotionRequest.walkRequest.request.rotation = (left - right) * parameters.maxVelocityRot;
    }
    else
      localMotionRequest.walkRequest.request.rotation = 0.0;
  }
}

void JoystickControl::generateCustomStep()
{
  localMotionRequest.walkRequest.stepRequest = currentExecutedCustomStep;
  localMotionRequest.kickRequest.mirror = currentExecutedCustomStepMirror;

  float stateValueX = 0.0f;
  float stateValueY = 0.0f;

  if (parameters.device.customStepXAxis >= 0 && static_cast<unsigned int>(parameters.device.customStepXAxis) <= parameters.device.axesCount)
  {
    stateValueX = state.axes[parameters.device.customStepXAxis];
  }

  if (parameters.device.customStepYAxis >= 0 && static_cast<unsigned int>(parameters.device.customStepYAxis) <= parameters.device.axesCount)
  {
    stateValueY = state.axes[parameters.device.customStepYAxis];
  }

  if (startOfCustomStep == 0)
  {
    currentExecutedCustomStep = WalkRequest::StepRequest::none;
    currentExecutedCustomStepMirror = false;

    for (JoystickDeviceParameters::StepRequestButtonPair pair : parameters.device.activeKicks)
    {
      if (pair.stateValueX == stateValueX && pair.stateValueY == stateValueY)
      {
        currentExecutedCustomStep = pair.stepRequest;
        currentExecutedCustomStepMirror = pair.mirror;

        startOfCustomStep = pair.duration;

        break;
      }
    }
  }
  else
  {
    startOfCustomStep -= 1;
  }
}

// --- Actions ---

void JoystickControl::stopMotion()
{
  localMotionRequest.motion = MotionRequest::walk;
  localMotionRequest.walkRequest.requestType = WalkRequest::speed;
  localMotionRequest.walkRequest.request.translation[0] = 0.f;
  localMotionRequest.walkRequest.request.translation[1] = 0.f;
  localMotionRequest.walkRequest.request.rotation = 0.0;
}

void JoystickControl::specialAction(SpecialActionRequest::SpecialActionID id, bool mirror, unsigned duration)
{
  localMotionRequest.motion = MotionRequest::specialAction;
  localMotionRequest.specialActionRequest.mirror = mirror;
  localMotionRequest.specialActionRequest.specialAction = id;
  // Set duration of action.
  m_actionEndTime = theFrameInfo.time + duration;
  m_actionRunning = true;
}

void JoystickControl::specialAction(SpecialActionRequest& specialAction, unsigned duration)
{
  localMotionRequest.motion = MotionRequest::specialAction;
  localMotionRequest.specialActionRequest = specialAction;
  // Set duration of action.
  m_actionEndTime = theFrameInfo.time + duration;
  m_actionRunning = true;
}


void JoystickControl::playDead()
{
  localMotionRequest.motion = MotionRequest::specialAction;
  localMotionRequest.specialActionRequest.mirror = false;
  localMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::playDead;
}

void JoystickControl::stand()
{
  specialAction(SpecialActionRequest::stand, false, 5000);
}

void JoystickControl::sitDown()
{
  specialAction(SpecialActionRequest::sitDown, false, 3000);
}

void JoystickControl::standUpFallen()
{
  // Set head angle.
  // This method has to be executed after generateHeadControlRequest() to set the head angles.
  // -> execute generateMotionRequest() after generateHeadControlRequest() in method execute().
  localHeadControlRequest.controlType = HeadControlRequest::direct;
  localHeadControlRequest.pan = 0;
  localHeadControlRequest.tilt = -0.3f;

  // Choose appropriate stand up action
  if (theFallDownState.state == FallDownState::onGround)
  {
    int duration = 1000;
    if (theFallDownState.direction == FallDownState::back)
    {
      specialAction(SpecialActionRequest::standUpBackNao, false, 5500);
      duration = 5500;
    }
    else if (theFallDownState.direction == FallDownState::front)
    {
      specialAction(SpecialActionRequest::standUpFrontNao, false, 3100);
      duration = 3100;
    }
    // Set duration of action.
    m_actionEndTime = theFrameInfo.time + duration;
    m_actionRunning = true;
  }
  else
  {
    // Prevent running other motions while the robot is falling and none of the
    // stand up actions can be executed. This causes, that the last motion
    // before falling is still executed until the stand up action starts.
    // Without this, the special action 'playDead' would be executed while the
    // robot is falling, what causes a delayed start of the stand up action.
    m_actionEndTime = theFrameInfo.time + 1000;
    m_actionRunning = true;
  }
}

void JoystickControl::kick(bool kickLeft)
{
  localMotionRequest.motion = MotionRequest::kick;
  localMotionRequest.kickRequest.mirror = !kickLeft;
  localMotionRequest.kickRequest.kickMotionType = KickRequest::kickMiddleFast;
  localMotionRequest.kickRequest.dynamical = true;
  localMotionRequest.kickRequest.kickTarget << 1000.f, 0.f;
  // Set duration of action.
  m_actionEndTime = theFrameInfo.time + 1000;
  m_actionRunning = true;
}
