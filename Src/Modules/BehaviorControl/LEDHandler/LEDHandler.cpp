/**
* @file LEDHandler.cpp
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff + alicia
*/

#include "LEDHandler.h"

#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest)
{
  // reset
  ledRequest = LEDRequest();

  // update blinking
  blinking = (theFrameInfo.time & 512) != 0 ? 1.f : 0.f;
  fastBlinking = (theFrameInfo.time & 128) != 0 ? 1.f : 0.f;

  // update
  setGameLEDs(ledRequest);

  // overrides for testing and calibration mode
  setTestLEDs(ledRequest);
  setCalibrationLEDs(ledRequest);

  // overrides for special cases
  setChargingStatus(ledRequest);
  setCheeringAnimation(ledRequest);

  setWhistleDetectionLEDs(ledRequest);
  setFlyingStateInfo(ledRequest);
  setDamagedJoints(ledRequest);

  // for testing: overwrite everything else
  // setFloatingFireEyes(ledRequest, 0, 30, 60);
  // setFloatingFireEyes(ledRequest, 240, 290, 340);
  // setStaticFireEyes(ledRequest, LEDRequest::RGBLED::red, LEDRequest::RGBLED::orange, LEDRequest::RGBLED::yellow);
  // setRandomizedFireEyes(ledRequest, LEDRequest::RGBLED::red, LEDRequest::RGBLED::orange, LEDRequest::RGBLED::yellow);
  // setFloatingRainbowEyes(ledRequest);
  // setStaticRainbowEyes(ledRequest);
}

void LEDHandler::setCalibrationLEDs(LEDRequest& ledRequest) const
{
  if (theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
  {
    ledRequest.chest = LEDRequest::RGBLED::magenta;

    if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::calibrateCameraMatrix)
    {
      if (theCMCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper || theCMCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureLower)
      {
        for (int i = 0; i < 12; ++i)
        {
          const float threshold = i / 12.f;
          ledRequest.head[i] = theCMCorrectorStatus.progress > threshold ? 1.0f : 0.0f;
        }
      }
      else
      {
        ledRequest.head.fill(blinking);
      }
    }
    else if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::calibrateBody)
    {
      if (!theWalkCalibration.bodyAngleCalibrated)
      {
        for (int i = 0; i < 12; ++i)
        {
          const float threshold = i / 12.f;
          ledRequest.head[i] = theWalkCalibration.bodyAngleProgress > threshold ? 1.0f : 0.0f;
        }
      }
      else
      {
        ledRequest.head.fill(blinking);
      }
    }

    else if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::calibrateWalk)
    {
      ledRequest.leftFoot = LEDRequest::RGBLED::fromHSV(static_cast<short>(theWalkCalibration.qualityOfRobotHardware * 60.f));
      ledRequest.rightFoot = ledRequest.leftFoot;
    }
  }
  else if (!theCameraCalibration.calibrated)
  {
    std::fill(ledRequest.leftEye.begin() + LEDRequest::EyeLED::eye135Deg, ledRequest.leftEye.begin() + LEDRequest::EyeLED::eye270Deg, LEDRequest::RGBLED::red);
    std::fill(ledRequest.rightEye.begin() + LEDRequest::EyeLED::eye135Deg, ledRequest.rightEye.begin() + LEDRequest::EyeLED::eye270Deg, LEDRequest::RGBLED::red);
  }
}

void LEDHandler::setTestLEDs(LEDRequest& ledRequest) const
{
  if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::testingJoints)
    ledRequest.chest = LEDRequest::RGBLED::cyan;
}

void LEDHandler::setWhistleDetectionLEDs(LEDRequest& ledRequest) const
{
  if (theWhistleDortmund.detectionState == WhistleDortmund::DetectionState::isDetected)
  {
    ledRequest.leftEar.fill(fastBlinking);
    ledRequest.rightEar.fill(fastBlinking);
    ledRequest.head.fill(fastBlinking);
  }
}

void LEDHandler::setGameLEDs(LEDRequest& ledRequest)
{
  setBatteryInfo(ledRequest);
  setConnectionInfo(ledRequest);
  setDetectionInfo(ledRequest);
  setRoleInfo(ledRequest);
  setGameStateInfo(ledRequest);
  setLocaInfo(ledRequest);
  setMotionInfo(ledRequest);
}

/** Battery info to right ear. */
void LEDHandler::setBatteryInfo(LEDRequest& ledRequest) const
{
  const size_t onLEDs = std::min(static_cast<size_t>(theSystemSensorData.batteryLevel / 0.1f), ledRequest.rightEar.size());
  std::fill(ledRequest.rightEar.begin(), ledRequest.rightEar.begin() + onLEDs, 1.f);
}

void LEDHandler::setChargingStatus(LEDRequest& ledRequest)
{
  if (theSystemSensorData.chargingStatus && theWalkCalibration.walkCalibrated)
  {
    //short baseColor = static_cast<short>(360.f - (1.f - theWalkCalibration.qualityOfRobotHardware) * 180.f); // red to blue
    short baseColor = static_cast<short>(350.f + theWalkCalibration.qualityOfRobotHardware * 120.f); // green to red
    setFloatingFireEyes(ledRequest, baseColor, baseColor + 20, baseColor + 40);
  }
}

void LEDHandler::setCheeringAnimation(LEDRequest& ledRequest)
{
  if (theMotionInfo.motion == MotionRequest::Motion::specialAction && theMotionInfo.specialActionRequest.specialAction >= SpecialActionRequest::SpecialActionID::cheering1
      && theMotionInfo.specialActionRequest.specialAction <= SpecialActionRequest::SpecialActionID::wave_left)
  {
    setFloatingRainbowEyes(ledRequest);
  }
}


/** Connection info to left ear. */
void LEDHandler::setConnectionInfo(LEDRequest& ledRequest) const
{
  const float gcState = theGameInfo.controllerConnected ? blinking : 1.f;
  for (size_t i = 0; i <= ledRequest.leftEar.size(); ++i)
  {
    const uint8_t num = static_cast<uint8_t>(std::round(float(i) * (MAX_NUM_PLAYERS - 1) / ledRequest.leftEar.size()));

    if (const TeammateReceived* mate = theTeammateData.getPlayer(num + 1); mate && mate->status != TeammateReceived::Status::INACTIVE)
    {
      ledRequest.leftEar[i] = gcState;
    }
  }
}

/** Set perception info to left eye. */
void LEDHandler::setDetectionInfo(LEDRequest& ledRequest) const
{
  if (theBallSymbols.ballWasSeen && theBallSymbols.timeSinceLastSeenByTeamMates < 3000)
    ledRequest.leftEye.fill(LEDRequest::RGBLED::white);
  else if (theBallSymbols.ballWasSeen)
    ledRequest.leftEye.fill(LEDRequest::RGBLED::green);
  else if (theBallSymbols.timeSinceLastSeenByTeamMates < 3000)
    ledRequest.leftEye.fill(LEDRequest::RGBLED::red);
}

/** Set role info to right eye. */
void LEDHandler::setRoleInfo(LEDRequest& ledRequest) const
{
  const float state = theBehaviorData.playerNumberToBall == theRobotInfo.number ? fastBlinking : 1.f;

  switch (theRoleSymbols.role)
  {
  case BehaviorData::keeper:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::blue * state);
    break;
  case BehaviorData::defenderLeft:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::green * state);
    ledRequest.rightEye[LEDRequest::EyeLED::eye225Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye270Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye315Deg] *= blinking;
    break;
  case BehaviorData::defenderRight:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::green * state);
    ledRequest.rightEye[LEDRequest::EyeLED::eye45Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye90Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye135Deg] *= blinking;
    break;
  case BehaviorData::backupBallchaser:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::magenta * state);
    break;
  case BehaviorData::defenderSingle:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::green * state);
    break;
  case BehaviorData::center:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::white * state);
    break;
  case BehaviorData::replacementKeeper:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::cyan * state);
    break;
  case BehaviorData::receiver:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::magenta * state);
    break;
  case BehaviorData::leftWing:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::orange * state);
    ledRequest.rightEye[LEDRequest::EyeLED::eye225Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye270Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye315Deg] *= blinking;
    break;
  case BehaviorData::rightWing:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::orange * state);
    ledRequest.rightEye[LEDRequest::EyeLED::eye45Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye90Deg] *= blinking;
    ledRequest.rightEye[LEDRequest::EyeLED::eye135Deg] *= blinking;
    break;
  case BehaviorData::frontWing:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::red * state);
    break;
  case BehaviorData::remoteControl:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::violet * state);
    break;
  case BehaviorData::backWing:
    ledRequest.rightEye.fill(LEDRequest::RGBLED::yellow * state);
    break;
  case BehaviorData::noRole:
    //off
    break;
  default:
    ASSERT(false);
  }
}

/** Set feedback for flying state to eyes. */
void LEDHandler::setFlyingStateInfo(LEDRequest& ledRequest)
{
  if (theFallDownState.state == FallDownState::flying)
  {
    setRotatingEyesTwoColors(ledRequest, LEDRequest::RGBLED::white, LEDRequest::RGBLED::black, 9, false);
  }
}

/** Set feedback for broken Joints to eyes. */
void LEDHandler::setDamagedJoints(LEDRequest& ledRequest)
{
  if (theBehaviorData.soccerState == BehaviorData::safetyShutdown)
  {
    setRandomizedFireEyes(ledRequest, LEDRequest::RGBLED::blue, LEDRequest::RGBLED::violet, LEDRequest::RGBLED::magenta);
  }
}

/** Set game state info to chest button. */
void LEDHandler::setGameStateInfo(LEDRequest& ledRequest)
{
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    ledRequest.chest = LEDRequest::RGBLED::red;
    setFloatingFireEyes(ledRequest, 0, 30, 60);
    return;
  }

  switch (theGameInfo.state)
  {
  case STATE_INITIAL:
    break;
  case STATE_STANDBY:
    ledRequest.chest = LEDRequest::RGBLED::cyan;
    break;
  case STATE_FINISHED:
    //if (theOwnTeamInfo.score > theOpponentTeamInfo.score)
    //  setFloatingRainbowEyes(ledRequest);
    break;
  case STATE_READY:
    ledRequest.chest = LEDRequest::RGBLED::blue;
    break;
  case STATE_SET:
    ledRequest.chest = LEDRequest::RGBLED::yellow;
    break;
  case STATE_PLAYING:
    ledRequest.chest = LEDRequest::RGBLED::green;
    break;
  default: //should not happen
    break;
  }
}

/** Set loca info on head. */
void LEDHandler::setLocaInfo(LEDRequest& ledRequest) const
{
  const float ledState = (theBehaviorData.soccerState == BehaviorData::controlBall) ? blinking : 1.f;
  if (theRobotPose.translation.x() > 0)
    ledRequest.head.fill(ledState);
  else
    std::fill(ledRequest.head.begin(), ledRequest.head.begin() + LEDRequest::HeadLED::middleRight0, ledState);
}

void LEDHandler::setMotionInfo(LEDRequest& ledRequest) const
{
  if (theStandEngineOutput.stiffnessTransition > 0.f && theStandEngineOutput.stiffnessTransition < 1.f)
  {
    ledRequest.leftFoot = LEDRequest::RGBLED::white * blinking;
    ledRequest.rightFoot = LEDRequest::RGBLED::white * blinking;
  }
  else if (theStandEngineOutput.stiffnessTransition >= 1.f)
  {
    ledRequest.leftFoot = LEDRequest::RGBLED::white;
    ledRequest.rightFoot = LEDRequest::RGBLED::white;
  }

  if (theSpeedInfo.lastCustomStepTimestamp > 0 && theFrameInfo.time < (theSpeedInfo.lastCustomStepTimestamp + 500))
  {
    auto& foot = theSpeedInfo.lastCustomStepMirrored ? ledRequest.rightFoot : ledRequest.leftFoot;

    switch (theSpeedInfo.lastCustomStep)
    {
    case WalkRequest::StepRequest::kickHack:
      foot = LEDRequest::RGBLED::yellow;
      break;
    case WalkRequest::StepRequest::kickHackLong:
      foot = LEDRequest::RGBLED::red;
      break;
    case WalkRequest::StepRequest::kickHackVeryLong:
      foot = LEDRequest::RGBLED::green;
      break;
    case WalkRequest::StepRequest::rotateKick45:
    case WalkRequest::StepRequest::keeperKick45:
      foot = LEDRequest::RGBLED::cyan;
      break;
    case WalkRequest::StepRequest::sideKickOuter45:
      foot = LEDRequest::RGBLED::blue;
      break;
    case WalkRequest::StepRequest::sideKickOuterFoot:
      foot = LEDRequest::RGBLED::magenta;
      break;
    case WalkRequest::StepRequest::sideKickOuterFront:
      foot = LEDRequest::RGBLED::white;
      break;
    }
  }
}

void LEDHandler::setMicStatusLEDs(LEDRequest& ledRequest)
{
  if (theGameInfo.state == STATE_INITIAL)
  {
  }
}


void LEDHandler::setRandomizedFireEyes(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED col3)
{
  if (lastUpdate % 5 == 0)
  {
    col1LEDs = {0, 0, (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), 1, (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0), 0};
    col2LEDs = {(rand() % 2 > 0), (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0), 1, 1, 1, (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0)};
    col3LEDs = {1, 1, 1, 1, 1, 1, 1, 1};
    lastUpdate = 0;
  }

  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
  {
    if (col1LEDs[i])
    {
      ledRequest.setBothEyes(i, col1, true);
    }
    else if (col2LEDs[i])
    {
      ledRequest.setBothEyes(i, col2, true);
    }
    else if (col3LEDs[i])
    {
      ledRequest.setBothEyes(i, col3, true);
    }
  }
  lastUpdate++;
}

void LEDHandler::setFloatingFireEyes(LEDRequest& ledRequest, short hue1, short hue2, short hue3)
{
  int updateRate = 20;
  if (lastUpdate % updateRate == 0)
  {
    col1LEDsOld = col1LEDs;
    col2LEDsOld = col2LEDs;
    col3LEDsOld = col3LEDs;

    col1LEDs = {0, 0, (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), 1, (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0), 0};
    col2LEDs = {(rand() % 2 > 0), (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0), 1, 1, 1, (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0)};
    col3LEDs = {1, 1, 1, 1, 1, 1, 1, 1};

    for (size_t i = 1; i < col1LEDs.size(); ++i)
    {
      if (i < 4)
      {
        if (col1LEDs.at(i) == 1)
        {
          col2LEDs.at(i + 1) = 1;
          col2LEDs.at(i + 2) = 1;
        }
        if (col1LEDsOld.at(i) == 1)
        {
          col1LEDs.at(i + 1) = 1;
        }
      }
      else if (i > 4)
      {
        if (col1LEDs.at(i) == 1)
        {
          col2LEDs.at(i - 1) = 1;
          col2LEDs.at(i - 2) = 1;
        }
        if (col1LEDsOld.at(i) == 1)
        {
          col1LEDs.at(i - 1) = 1;
        }
      }
    }

    lastUpdate = 0;
  }

  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
  {
    if (col1LEDs[i])
    {
      if (col1LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue1), true);
      }
      else if (col2LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue2 - static_cast<short>((lastUpdate + 1) * (hue2 - hue1) / updateRate)), true);
      }
      else
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue3 - static_cast<short>((lastUpdate + 1) * (hue3 - hue1) / updateRate)), true);
      }
    }
    else if (col2LEDs[i])
    {
      if (col1LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue1 + static_cast<short>((lastUpdate + 1) * (hue2 - hue1) / updateRate)), true);
      }
      else if (col2LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue2), true);
      }
      else
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue3 + static_cast<short>((lastUpdate + 1) * (hue2 - hue3) / updateRate)), true);
      }
    }
    else
    {
      if (col1LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue1 + static_cast<short>((lastUpdate + 1) * (hue3 - hue1) / updateRate)), true);
      }
      else if (col2LEDsOld[i])
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue2 + static_cast<short>((lastUpdate + 1) * (hue3 - hue2) / updateRate)), true);
      }
      else
      {
        ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(hue3), true);
      }
    }
  }
  lastUpdate++;
}

void LEDHandler::setStaticFireEyes(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED col3)
{
  col1LEDs = {0, 0, 0, 0, 1, 1, 1, 0};
  col2LEDs = {0, 1, 1, 1, 0, 0, 0, 1};
  col3LEDs = {1, 0, 0, 0, 0, 0, 0, 0};

  if (rotationStateFire < 3)
  {
    col1LEDs = {0, 0, 0, 0, 1, 0, 0, 0};
    col2LEDs = {0, 0, 1, 1, 0, 1, 1, 0};
    col3LEDs = {1, 1, 0, 0, 0, 0, 0, 1};
    rotationStateFire++;
  }
  else if (rotationStateFire < 8)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 1, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationStateFire++;
  }
  else if (rotationStateFire < 13)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {1, 1, 0, 0, 0, 0, 0, 0};
    rotationStateFire++;
  }
  else if (rotationStateFire < 18)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationStateFire++;
  }
  else if (rotationStateFire < 23)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {1, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    rotationStateFire++;
  }
  else if (rotationStateFire < 28)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 1, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationStateFire++;
  }
  else if (rotationStateFire < 33)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {1, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    rotationStateFire++;
  }
  else if (rotationStateFire < 38)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationStateFire++;
  }
  else if (rotationStateFire < 43)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 1, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationStateFire = rotationStateFire == 42 ? 0 : rotationStateFire + 1;
  }

  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
  {
    if (col1LEDs[i])
    {
      ledRequest.setBothEyes(i, col1, true);
    }
    else if (col2LEDs[i])
    {
      ledRequest.setBothEyes(i, col2, true);
    }
    else if (col3LEDs[i])
    {
      ledRequest.setBothEyes(i, col3, true);
    }
  }
}

void LEDHandler::setStaticRainbowEyes(LEDRequest& ledRequest)
{
  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
    ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(static_cast<short>(i * 45), 1.f, 1.f));
}

void LEDHandler::setFloatingRainbowEyes(LEDRequest& ledRequest)
{
  offset = (offset + 10) % 360;

  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
    ledRequest.setBothEyes(i, LEDRequest::RGBLED::fromHSV(static_cast<short>((i * 45 + offset) % 360), 1.f, 1.f));
}

void LEDHandler::setRotatingEyesTwoColors(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, int duration, bool mirrored)
{
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> firstColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};

  if (rotationState < duration)
  {
    firstColorLEDs = {1, 0, 1, 0, 1, 0, 1, 0};
    rotationState++;
  }
  else if (rotationState < 2 * duration)
  {
    firstColorLEDs = {0, 1, 0, 1, 0, 1, 0, 1};
    rotationState = rotationState >= 2 * duration - 1 ? 0 : rotationState + 1;
  }

  for (size_t i = 0; i < ledRequest.leftEye.size(); ++i)
  {
    if (firstColorLEDs[i])
    {
      ledRequest.setBothEyes(i, col1, mirrored);
    }
    else
    {
      ledRequest.setBothEyes(i, col2, mirrored);
    }
  }
}

void LEDHandler::setRotatingEyesThreeColors(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED transition, int duration, bool mirrored)
{
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> firstColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> secondColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> transitionColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};

  if (rotationState < duration)
  {
    firstColorLEDs = {1, 0, 0, 0, 1, 0, 0, 0};
    secondColorLEDs = {0, 0, 1, 0, 0, 0, 1, 0};
    transitionColorLEDs = {0, 1, 0, 1, 0, 1, 0, 1};
    rotationState++;
  }
  else if (rotationState < 2 * duration)
  {
    firstColorLEDs = {0, 1, 0, 0, 0, 1, 0, 0};
    secondColorLEDs = {0, 0, 0, 1, 0, 0, 0, 1};
    transitionColorLEDs = {1, 0, 1, 0, 1, 0, 1, 0};
    rotationState++;
  }
  else if (rotationState < 3 * duration)
  {
    firstColorLEDs = {0, 0, 1, 0, 0, 0, 1, 0};
    secondColorLEDs = {1, 0, 0, 0, 1, 0, 0, 0};
    transitionColorLEDs = {0, 1, 0, 1, 0, 1, 0, 1};
    rotationState++;
  }
  else if (rotationState < 4 * duration)
  {
    firstColorLEDs = {0, 0, 0, 1, 0, 0, 0, 1};
    secondColorLEDs = {0, 1, 0, 0, 0, 1, 0, 0};
    transitionColorLEDs = {1, 0, 1, 0, 1, 0, 1, 0};
    rotationState = rotationState >= 4 * duration - 1 ? 0 : rotationState + 1;
  }

  for (size_t i = 0; i < ledRequest.leftEye.size(); i++)
  {
    if (firstColorLEDs[i])
    {
      ledRequest.setBothEyes(i, col1, mirrored);
    }
    else if (secondColorLEDs[i])
    {
      ledRequest.setBothEyes(i, col2, mirrored);
    }
    else if (transitionColorLEDs[i])
    {
      ledRequest.setBothEyes(i, transition, mirrored);
    }
  }
}

MAKE_MODULE(LEDHandler, behaviorControl)
