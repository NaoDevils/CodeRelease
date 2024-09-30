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

JoystickControl::JoystickControl() : m_walkOnSpot(false)
{
  initialize();
}

void JoystickControl::update(ArmContact& armContact)
{
  armContact.armContactStateLeft = ArmContact::None;
  armContact.armContactStateRight = ArmContact::None;
  armContact.timeStampLeft = 0;
  armContact.timeStampRight = 0;
}

void JoystickControl::update(HeadControlRequest& headControlRequest)
{
  headControlRequest = localHeadControlRequest;
}

void JoystickControl::update(MotionRequest& motionRequest)
{
  motionRequest = localMotionRequest;
}

void JoystickControl::initialize()
{
  // Robot starts with centered head.
  localHeadControlRequest.controlType = HeadControlRequest::direct;
  localHeadControlRequest.pan = 0_deg;
  localHeadControlRequest.tilt = 0_deg;

  // Robot starts playing dead.
  m_standing = false;
  playDead();
  localMotionRequest.walkRequest.requestType = WalkRequest::speed;
  localMotionRequest.walkRequest.request.translation << 0.f, 0.f;
  localMotionRequest.walkRequest.request.rotation = 0_deg;
  localMotionRequest.kickRequest.kickMotionType = KickRequest::none;

  // No action running.
  m_actionRunning = false;
  m_actionEndTime = 0;
}

void JoystickControl::execute(tf::Subflow&)
{
  checkButtonOrAxisAvailability();

  if (backButtonAvailable && theJoystickState.wasButtonReleasedThisFrame(theJoystickDeviceParameters.backButton))
  {
    m_walkOnSpot = !m_walkOnSpot;
  }

  if (theRobotInfo.transitionToFramework >= 0.f)
  {
    // Process joystick events.
    generateHeadControlRequest();
    generateMotionRequest();

    // Stay in walking when special mode is on or last walk motion is less than 10 seconds old
    bool walkingActive = localMotionRequest.motion == MotionRequest::walk && localMotionRequest.walkRequest.requestType == WalkRequest::speed
        && (localMotionRequest.walkRequest.request.translation.norm() > 0.f || std::abs(localMotionRequest.walkRequest.request.rotation) > 0_deg);
    if (walkingActive)
    {
      m_timestampOfLastWalkMovement = theFrameInfo.time;
    }
    if (!walkingActive)
    {
      if ((m_walkOnSpot || theFrameInfo.getTimeSince(m_timestampOfLastWalkMovement) < static_cast<int>(continueWalkingTime)))
      {
        localMotionRequest.walkRequest.request.translation.x() = -1.f;
      }
      if (standing() && !actionRunning() && theFrameInfo.getTimeSince(m_timestampOfLastWalkMovement) >= static_cast<int>(continueWalkingTime))
      {
        localMotionRequest.motion = MotionRequest::stand;
      }
    }
  }
  else
  {
    m_standing = false;
    playDead();
  }
}

void JoystickControl::processStartButton()
{
  // Was the start button pressed?
  if (startButtonAvailable && theJoystickState.wasButtonReleasedThisFrame(theJoystickDeviceParameters.startButton))
  {
    // Change standing theJoystickState.
    if (standing())
    {
      m_standing = false;
      sitDown();
    }
    else
    {
      m_standing = true;
      stand();
    }
  }
}

void JoystickControl::generateHeadControlRequest()
{
  localHeadControlRequest.controlType = HeadControlRequest::direct;

  // Set pan.
  if (headPanAxisAvailable)
    localHeadControlRequest.pan = -theJoystickState.axes[theJoystickDeviceParameters.headPanAxis] * theJoystickControlParameters.maxHeadPan;
  else
    localHeadControlRequest.pan = 0.f;

  // Set tilt.
  if (headTiltAxisAvailable)
  {
    float maxWholeTiltAngle = std::abs(theJoystickControlParameters.maxHeadTilt) + std::abs(theJoystickControlParameters.minHeadTilt);
    localHeadControlRequest.tilt = -theJoystickState.axes[theJoystickDeviceParameters.headTiltAxis] * (maxWholeTiltAngle / 2.f);
    localHeadControlRequest.tilt += theJoystickControlParameters.minHeadTilt + (maxWholeTiltAngle / 2.f);

    if (localHeadControlRequest.tilt < theJoystickControlParameters.minHeadTilt)
      localHeadControlRequest.tilt = theJoystickControlParameters.minHeadTilt;
    if (localHeadControlRequest.tilt > theJoystickControlParameters.maxHeadTilt)
      localHeadControlRequest.tilt = theJoystickControlParameters.maxHeadTilt;
  }
  else
    localHeadControlRequest.tilt = 0.f;
}

void JoystickControl::generateMotionRequest()
{
  // Keep last motion request while the action is still running.
  if (actionRunning())
  {
    return;
  }

  processStartButton();

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

  // Evaluate button press if available.
  if (theFrameInfo.getTimeSince(theJoystickState.timestampLastButtonEvent) < (theFrameInfo.cycleTime * 1000))
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
  if (leftKickButtonAvailable)
  {
    if (theJoystickState.wasButtonReleasedThisFrame(theJoystickDeviceParameters.leftKickButton))
    {
      kick(true);
      return;
    }
  }

  // Trigger right kick.
  if (rightKickButtonAvailable)
  {
    if (theJoystickState.wasButtonReleasedThisFrame(theJoystickDeviceParameters.rightKickButton))
    {
      kick(false);
      return;
    }
  }

  // Trigger special actions.
  for (size_t s = 0; s < theJoystickDeviceParameters.specialActions.size(); s++)
  {
    if (specialActionsButtonAvailable.at(s))
    {
      if (theJoystickState.wasButtonReleasedThisFrame(theJoystickDeviceParameters.specialActions[s].button))
      {
        specialAction(theJoystickDeviceParameters.specialActions[s].specialAction, theJoystickDeviceParameters.specialActions[s].duration);
        return;
      }
    }
  }
}

bool JoystickControl::standing()
{
  if (!m_standing)
    // The robot is sitting. Press start button to stand up.
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
  // Walk at speed.
  localMotionRequest.motion = MotionRequest::walk;
  localMotionRequest.walkRequest.requestType = WalkRequest::speed;

  // Walking direction x.
  if (walkXAxisAvailable)
    localMotionRequest.walkRequest.request.translation[0] = -theJoystickState.axes[theJoystickDeviceParameters.walkXAxis] * theJoystickControlParameters.maxVelocityX;
  else
    localMotionRequest.walkRequest.request.translation[0] = 0.f;

  // Add value of additional walkXAxis.
  if (additionalWalkXAxisAvailable)
  {
    localMotionRequest.walkRequest.request.translation[0] = localMotionRequest.walkRequest.request.translation.x()
        - theJoystickState.axes[theJoystickDeviceParameters.additionalWalkXAxis] * theJoystickControlParameters.maxVelocityX;
  }
  if (localMotionRequest.walkRequest.request.translation.x() > 0.0 && localMotionRequest.walkRequest.request.translation.x() > theJoystickControlParameters.maxVelocityX)
    localMotionRequest.walkRequest.request.translation[0] = theJoystickControlParameters.maxVelocityX;
  if (localMotionRequest.walkRequest.request.translation.x() < 0.0 && localMotionRequest.walkRequest.request.translation.x() < theJoystickControlParameters.minVelocityX)
    localMotionRequest.walkRequest.request.translation[0] = theJoystickControlParameters.minVelocityX;

  // Walking direction y.
  if (walkYAxisAvailable)
    localMotionRequest.walkRequest.request.translation[1] = -theJoystickState.axes[theJoystickDeviceParameters.walkYAxis] * theJoystickControlParameters.maxVelocityY;
  else
    localMotionRequest.walkRequest.request.translation[1] = 0.f;

  // Walking rotation.
  if (walkRotAxisAvailable)
    localMotionRequest.walkRequest.request.rotation = -theJoystickState.axes[theJoystickDeviceParameters.walkRotAxis] * theJoystickControlParameters.maxVelocityRot;
  else if (theJoystickDeviceParameters.walkRotAxisSplit())
  {
    float left = (theJoystickState.axes[theJoystickDeviceParameters.walkRotLeftAxis] + 1.f) / 2.f; // range [0,1]
    float right = (theJoystickState.axes[theJoystickDeviceParameters.walkRotRightAxis] + 1.f) / 2.f; // range [0,1]
    localMotionRequest.walkRequest.request.rotation = (left - right) * theJoystickControlParameters.maxVelocityRot;
  }
  else
    localMotionRequest.walkRequest.request.rotation = 0.0;
}

void JoystickControl::generateCustomStep()
{
  localMotionRequest.walkRequest.stepRequest = currentExecutedCustomStep;
  localMotionRequest.kickRequest.mirror = currentExecutedCustomStepMirror;

  float stateValueX = 0.0f;
  float stateValueY = 0.0f;


  if (customStepXAxisAvailable)
  {
    stateValueX = theJoystickState.axes[theJoystickDeviceParameters.customStepXAxis];
  }

  if (customStepYAxisAvailable)
  {
    stateValueY = theJoystickState.axes[theJoystickDeviceParameters.customStepYAxis];
  }

  if (startOfCustomStep == 0)
  {
    currentExecutedCustomStep = WalkRequest::StepRequest::none;
    currentExecutedCustomStepMirror = false;

    for (JoystickDeviceParameters::StepRequestButtonPair pair : theJoystickDeviceParameters.activeKicks)
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

void JoystickControl::checkButtonOrAxisAvailability()
{
  // clang-format off
  startButtonAvailable = theJoystickDeviceParameters.startButton >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.startButton) <= theJoystickDeviceParameters.buttonsCount;
  backButtonAvailable = theJoystickDeviceParameters.backButton >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.backButton) <= theJoystickDeviceParameters.buttonsCount;
  rightKickButtonAvailable = theJoystickDeviceParameters.rightKickButton >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.rightKickButton) <= theJoystickDeviceParameters.buttonsCount;
  leftKickButtonAvailable = theJoystickDeviceParameters.leftKickButton >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.leftKickButton) <= theJoystickDeviceParameters.buttonsCount;

  specialActionsButtonAvailable.clear();
  for (size_t s = 0; s < theJoystickDeviceParameters.specialActions.size(); s++)
    specialActionsButtonAvailable.push_back(theJoystickDeviceParameters.specialActions[s].button >= 0
        && static_cast<unsigned int>(theJoystickDeviceParameters.specialActions[s].button) <= theJoystickDeviceParameters.buttonsCount);

  headPanAxisAvailable = theJoystickDeviceParameters.headPanAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.headPanAxis) <= theJoystickDeviceParameters.axesCount;
  headTiltAxisAvailable = theJoystickDeviceParameters.headTiltAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.headTiltAxis) <= theJoystickDeviceParameters.axesCount;
  walkXAxisAvailable = theJoystickDeviceParameters.walkXAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.walkXAxis) <= theJoystickDeviceParameters.axesCount;
  additionalWalkXAxisAvailable = theJoystickDeviceParameters.additionalWalkXAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.additionalWalkXAxis) <= theJoystickDeviceParameters.axesCount;
  walkYAxisAvailable = theJoystickDeviceParameters.walkYAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.walkYAxis) <= theJoystickDeviceParameters.axesCount;
  walkRotAxisAvailable = theJoystickDeviceParameters.walkRotAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.walkRotAxis) <= theJoystickDeviceParameters.axesCount;
  customStepXAxisAvailable = theJoystickDeviceParameters.customStepXAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.customStepXAxis) <= theJoystickDeviceParameters.axesCount;
  customStepYAxisAvailable = theJoystickDeviceParameters.customStepYAxis >= 0 && static_cast<unsigned int>(theJoystickDeviceParameters.customStepYAxis) <= theJoystickDeviceParameters.axesCount;
  // clang-format on
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

void JoystickControl::specialAction(const SpecialActionRequest& specialAction, unsigned duration)
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
  specialAction(SpecialActionRequest::stand, false, standTime);
  //specialAction(SpecialActionRequest::standHigh, false, standTime);
}

void JoystickControl::sitDown()
{
  specialAction(SpecialActionRequest::sitDown, false, sitDownTime);
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
    if (theFallDownState.direction == FallDownState::back)
    {
      specialAction(SpecialActionRequest::standUpBack, false, standUpBackTime);
    }
    else if (theFallDownState.direction == FallDownState::front)
    {
      specialAction(SpecialActionRequest::standUpFront, false, standUpFrontTime);
    }
    // Set the action running.
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
  m_actionEndTime = theFrameInfo.time + kickTime;
  m_actionRunning = true;
}
