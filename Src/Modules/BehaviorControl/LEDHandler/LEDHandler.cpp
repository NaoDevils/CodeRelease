/**
* @file LEDHandler.cpp
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#include "LEDHandler.h"

#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest)
{
  //reset
  for (int i = 0; i < ledRequest.numOfLEDs; ++i)
    ledRequest.ledStates[i] = LEDRequest::off;

  //update
  if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::frameworkInactive)
    setFrameworkInactiveLEDs(ledRequest);
  else if (theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
    setCalibrationLEDs(ledRequest);
  else
  {
    setGameLEDs(ledRequest);
  }
  // whistle detection always overrides stuff
  setWhistleDetectionLEDs(ledRequest);
}

void LEDHandler::setFrameworkInactiveLEDs(LEDRequest& ledRequest) {}

void LEDHandler::setCalibrationLEDs(LEDRequest& ledRequest)
{
  // set chest to purple as per GORE 2021 rules
  ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
  ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;

  // use some of the default LEDs
  setDetectionInfo(ledRequest);
  setBatteryInfo(ledRequest);
  setLocaInfo(ledRequest);
  // left ear only displays (gc) connection info, not teammate info
  LEDRequest::LEDState gcState = (theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > 2000) ? LEDRequest::blinking : LEDRequest::on;
  for (int i = 0; i < 5; i++)
  {
    ledRequest.ledStates[LEDRequest::earsLeft0Deg + 2 * i] = gcState;
    ledRequest.ledStates[LEDRequest::earsLeft36Deg + 2 * i + 1] = LEDRequest::on;
  }

  if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::calibrateCameraMatrix)
  {
    if (theCMCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureUpper || theCMCorrectorStatus.state == CMCorrectorStatus::CalibrationState::captureLower)
    {
      const float progress = theCMCorrectorStatus.progress;

      for (int i = 0; i < 12; ++i)
      {
        const float threshold = i / 12.f;
        ledRequest.ledStates[LEDRequest::LED::headLedRearLeft0 + i] = progress > threshold ? LEDRequest::LEDState::on : LEDRequest::LEDState::off;
      }
    }
    else
    {
      for (int i = 0; i < 12; ++i)
        ledRequest.ledStates[LEDRequest::LED::headLedRearLeft0 + i] = LEDRequest::LEDState::blinking;
    }
  }
}

void LEDHandler::setWhistleDetectionLEDs(LEDRequest& ledRequest)
{
  if (theWhistleDortmund.detectionState == WhistleDortmund::DetectionState::isDetected)
  {
    for (int i = 0; i < 10; i++)
    {
      ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = LEDRequest::fastBlinking;
      ledRequest.ledStates[LEDRequest::earsLeft0Deg + i] = LEDRequest::fastBlinking;
    }
    for (unsigned i = LEDRequest::headLedRearLeft0; i <= LEDRequest::headLedMiddleLeft0; i++)
      ledRequest.ledStates[i] = LEDRequest::fastBlinking;
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
void LEDHandler::setBatteryInfo(LEDRequest& ledRequest)
{
  int onLEDs = std::min(static_cast<int>(theSystemSensorData.batteryLevel / 0.1f), 10);
  for (int i = 0; i < onLEDs; i++)
    ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = LEDRequest::on;
}

/** Connection info to left ear. */
void LEDHandler::setConnectionInfo(LEDRequest& ledRequest)
{
  LEDRequest::LEDState gcState = (theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > 2000) ? LEDRequest::blinking : LEDRequest::on;

  int numberOfConnectedTeammates = std::min(static_cast<int>(theTeammateData.teammates.size()), 4);
  for (int i = 0; i < numberOfConnectedTeammates + 1; i++)
  {
    ledRequest.ledStates[LEDRequest::earsLeft0Deg + 2 * i] = gcState;
    ledRequest.ledStates[LEDRequest::earsLeft36Deg + 2 * i + 1] = LEDRequest::on;
  }
}

/** Set perception info to left eye. */
void LEDHandler::setDetectionInfo(LEDRequest& ledRequest)
{
  if (theBallSymbols.ballWasSeen && theBallSymbols.timeSinceLastSeenByTeamMates < 3000)
    setEyeColor(ledRequest, true, BehaviorLEDRequest::white, LEDRequest::on);
  else if (theBallSymbols.ballWasSeen)
    setEyeColor(ledRequest, true, BehaviorLEDRequest::green, LEDRequest::on);
  else if (theBallSymbols.timeSinceLastSeenByTeamMates < 3000)
    setEyeColor(ledRequest, true, BehaviorLEDRequest::red, LEDRequest::on);
  else
  {
    if (theBallModel.timeWhenLastSeen == theFrameInfo.time)
      setEyeColor(ledRequest, true, BehaviorLEDRequest::magenta, LEDRequest::on);
  }
}

/** Set role info to right eye. */
void LEDHandler::setRoleInfo(LEDRequest& ledRequest)
{
  switch (theRoleSymbols.role)
  {
  case BehaviorData::keeper:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, LEDRequest::on);
    break;
  case BehaviorData::defenderLeft:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::green, LEDRequest::on, true);
    break;
  case BehaviorData::defenderRight:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::green, LEDRequest::on, false, true);
    break;
  case BehaviorData::ballchaserKeeper:
  case BehaviorData::ballchaser:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::red, LEDRequest::on);
    break;
  case BehaviorData::backupBallchaser:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, LEDRequest::on);
    break;
  case BehaviorData::defenderSingle:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::yellow, LEDRequest::on);
    break;
  case BehaviorData::center:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::white, LEDRequest::on);
    break;
  case BehaviorData::replacementKeeper:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::cyan, LEDRequest::on);
    break;
  case BehaviorData::receiver:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, LEDRequest::blinking);
    break;
  case BehaviorData::noRole:
    //off
    break;
  default:
    ASSERT(false);
  }
}

/** Set game state info to chest button. */
void LEDHandler::setGameStateInfo(LEDRequest& ledRequest)
{
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    return;
  }

  switch (theGameInfo.state)
  {
  case STATE_INITIAL:
  case STATE_FINISHED:
  {
    // leave off
    break;
  }
  case STATE_READY:
  {
    ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
    break;
  }
  case STATE_SET:
  {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
    break;
  }
  case STATE_PLAYING:
  {
    ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;
    break;
  }
  default: //should not happen
    break;
  }
}

/** Set loca info on head. */
void LEDHandler::setLocaInfo(LEDRequest& ledRequest)
{
  LEDRequest::LEDState ledState = (theBehaviorData.soccerState == BehaviorData::controlBall) ? LEDRequest::blinking : LEDRequest::on;
  if (theRobotPose.translation.x() > 0)
  {
    for (unsigned i = LEDRequest::headLedRearLeft0; i <= LEDRequest::headLedMiddleLeft0; i++)
      ledRequest.ledStates[i] = ledState;
  }
  else
  {
    for (unsigned i = LEDRequest::headLedRearLeft0; i < LEDRequest::headLedMiddleRight0; i++)
      ledRequest.ledStates[i] = ledState;
    for (unsigned i = LEDRequest::headLedMiddleRight0; i <= LEDRequest::headLedMiddleLeft0; i++)
      ledRequest.ledStates[i] = LEDRequest::off;
  }
}

/** Set obstacle info to feet. */
void LEDHandler::setObstacleInfo(LEDRequest& ledRequest) {}

void LEDHandler::setEyeColor(LEDRequest& ledRequest, bool left, BehaviorLEDRequest::EyeColor col, LEDRequest::LEDState s, bool onlyLeft, bool onlyRight)
{
  LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

  static const int redOffset = 0, greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg, blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg;

  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;
  std::vector<bool> useLED = {1, 1, 1, 1, 1, 1, 1, 1};
  if (onlyLeft)
    useLED = {1, 1, 1, 1, 1, 0, 0, 0};
  if (onlyRight)
    useLED = {1, 0, 0, 0, 1, 1, 1, 1};

  LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

  switch (col)
  {
  case BehaviorLEDRequest::defaultColor:
    ASSERT(false);
    break;
  case BehaviorLEDRequest::red:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::green:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::blue:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::white:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::magenta:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::yellow:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? halfState : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::cyan:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? halfState : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::orange:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  default:
    ASSERT(false);
    break;
  }
}

void LEDHandler::setMotionInfo(LEDRequest& ledRequest)
{
  if (theStandEngineOutput.stiffnessTransition > 0.f && theStandEngineOutput.stiffnessTransition < 1.f)
  {
    for (int i = LEDRequest::footLeftRed; i <= LEDRequest::footRightBlue; i++)
      ledRequest.ledStates[i] = LEDRequest::blinking;
  }
  else if (theStandEngineOutput.stiffnessTransition >= 1.f)
  {
    for (int i = LEDRequest::footLeftRed; i <= LEDRequest::footRightBlue; i++)
      ledRequest.ledStates[i] = LEDRequest::on;
  }

  if (theSpeedInfo.lastCustomStepTimestamp > 0 && theFrameInfo.time < (theSpeedInfo.lastCustomStepTimestamp + 500))
  {
    int r, g, b;
    if (!theSpeedInfo.lastCustomStepMirrored)
    {
      r = LEDRequest::footLeftRed;
      g = LEDRequest::footLeftGreen;
      b = LEDRequest::footLeftBlue;
    }
    else
    {
      r = LEDRequest::footRightRed;
      g = LEDRequest::footRightGreen;
      b = LEDRequest::footRightBlue;
    }

    switch (theSpeedInfo.lastCustomStep)
    {
    case WalkRequest::StepRequest::kickHack: // YELLOW
      ledRequest.ledStates[r] = LEDRequest::on;
      ledRequest.ledStates[g] = LEDRequest::on;
      ledRequest.ledStates[b] = LEDRequest::off;
      break;
    case WalkRequest::StepRequest::kickHackLong: // RED
      ledRequest.ledStates[r] = LEDRequest::on;
      ledRequest.ledStates[g] = LEDRequest::off;
      ledRequest.ledStates[b] = LEDRequest::off;
      break;
    case WalkRequest::StepRequest::kickHackVeryLong: // GREEN
      ledRequest.ledStates[r] = LEDRequest::off;
      ledRequest.ledStates[g] = LEDRequest::on;
      ledRequest.ledStates[b] = LEDRequest::off;
      break;
    case WalkRequest::StepRequest::rotateKick45: // GREEN
      ledRequest.ledStates[r] = LEDRequest::off;
      ledRequest.ledStates[g] = LEDRequest::on;
      ledRequest.ledStates[b] = LEDRequest::off;
      break;
    case WalkRequest::StepRequest::sideKickOuter45: // BLUE
      ledRequest.ledStates[r] = LEDRequest::off;
      ledRequest.ledStates[g] = LEDRequest::off;
      ledRequest.ledStates[b] = LEDRequest::on;
      break;
    case WalkRequest::StepRequest::sideKickOuterFoot: // PURPLE
      ledRequest.ledStates[r] = LEDRequest::on;
      ledRequest.ledStates[g] = LEDRequest::off;
      ledRequest.ledStates[b] = LEDRequest::on;
      break;
    case WalkRequest::StepRequest::sideKickOuterFront: // WHITE
      ledRequest.ledStates[r] = LEDRequest::on;
      ledRequest.ledStates[g] = LEDRequest::on;
      ledRequest.ledStates[b] = LEDRequest::on;
      break;
    }
  }
}

MAKE_MODULE(LEDHandler, behaviorControl)
