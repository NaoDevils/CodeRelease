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
  else if (theBehaviorData.behaviorState == BehaviorData::BehaviorState::testingJoints)
  {
    setTestLEDs(ledRequest);
  }
  else
  {
    setGameLEDs(ledRequest);
  }
  // whistle detection always overrides stuff
  setWhistleDetectionLEDs(ledRequest);
  // flying state as well
  setFlyingStateInfo(ledRequest);
  setDamagedJoints(ledRequest);

  if (theSystemSensorData.chargingStatus)
  {
    if (theSystemSensorData.batteryLevel > 0.95f)
    {
      setRandomizedFireEyes(ledRequest, BehaviorLEDRequest::darkred, BehaviorLEDRequest::darkyellow, BehaviorLEDRequest::yellow);
    }
    else
    {
      setRandomizedFireEyes(ledRequest, BehaviorLEDRequest::darkblue, BehaviorLEDRequest::cyan, BehaviorLEDRequest::white);
    }
    // setDynamicRainbowEyes(ledRequest);
    // setStaticRainbowEyes(ledRequest);
    // setRotatingEyesTwoColors(ledRequest, BehaviorLEDRequest::blue, BehaviorLEDRequest::magenta, 5);
    // setRotatingEyesThreeColors(ledRequest, BehaviorLEDRequest::red, BehaviorLEDRequest::yellow, BehaviorLEDRequest::orange, 5); // fire eyes
  }
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
void LEDHandler::setTestLEDs(LEDRequest& ledRequest)
{
  // set chest to purple as per GORE 2021 rules
  ledRequest.ledStates[LEDRequest::chestBlue] = LEDRequest::on;
  ledRequest.ledStates[LEDRequest::chestGreen] = LEDRequest::on;

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

void LEDHandler::setRandomizedFireEyes(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor col3)
{
  if (lastUpdate % 7 == 0)
  {
    col1LEDs = {0, 0, (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), 1, (rand() % 2 > 0) || (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0), 0};
    col2LEDs = {(rand() % 2 > 0), (rand() % 2 > 0), (rand() % 2 > 0) || (rand() % 2 > 0), 1, 1, 1, (rand() % 2 > 0) || (rand() % 2 > 0), (rand() % 2 > 0)};
    col3LEDs = {1, 1, 1, 1, 1, 1, 1, 1};
    lastUpdate = 0;
  }

  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;
  for (int i = 0; i < numOfLEDsPerColor; i++)
  {
    if (col1LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col1, i, true);
    }
    else if (col2LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col2, i, true);
    }
    else if (col3LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col3, i, true);
    }
  }

  lastUpdate++;
}

void LEDHandler::setStaticFireEyes(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor col3)
{
  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;

  col1LEDs = {0, 0, 0, 0, 1, 1, 1, 0};
  col2LEDs = {0, 1, 1, 1, 0, 0, 0, 1};
  col3LEDs = {1, 0, 0, 0, 0, 0, 0, 0};

  if (rotationState < 3)
  {
    col1LEDs = {0, 0, 0, 0, 1, 0, 0, 0};
    col2LEDs = {0, 0, 1, 1, 0, 1, 1, 0};
    col3LEDs = {1, 1, 0, 0, 0, 0, 0, 1};
    rotationState++;
  }
  else if (rotationState < 8)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 1, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationState++;
  }
  else if (rotationState < 13)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {1, 1, 0, 0, 0, 0, 0, 0};
    rotationState++;
  }
  else if (rotationState < 18)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationState++;
  }
  else if (rotationState < 23)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {1, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    rotationState++;
  }
  else if (rotationState < 28)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 1, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationState++;
  }
  else if (rotationState < 33)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {1, 0, 1, 0, 0, 0, 1, 1};
    col3LEDs = {0, 1, 0, 0, 0, 0, 0, 0};
    rotationState++;
  }
  else if (rotationState < 38)
  {
    col1LEDs = {0, 0, 1, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 0, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationState++;
  }
  else if (rotationState < 43)
  {
    col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
    col2LEDs = {0, 1, 1, 0, 0, 0, 1, 0};
    col3LEDs = {1, 0, 0, 0, 0, 0, 0, 1};
    rotationState = rotationState == 42 ? 0 : rotationState + 1;
  }

  for (int i = 0; i < numOfLEDsPerColor; i++)
  {
    if (col1LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col1, i, true);
    }
    else if (col2LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col2, i, true);
    }
    else if (col3LEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col3, i, true);
    }
  }
}

void LEDHandler::setDynamicRainbowEyes(LEDRequest& ledRequest)
{
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::white, (rotationState / 10) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::yellow, (rotationState / 10 + 1) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::orange, (rotationState / 10 + 2) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::red, (rotationState / 10 + 3) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::magenta, (rotationState / 10 + 4) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::cyan, (rotationState / 10 + 5) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::blue, (rotationState / 10 + 6) % 8, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::green, (rotationState / 10 + 7) % 8, true);
  rotationState++;
  if (rotationState == 79)
    rotationState = 0;
}

void LEDHandler::setStaticRainbowEyes(LEDRequest& ledRequest)
{
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::white, 0, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::yellow, 1, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::orange, 2, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::red, 3, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::magenta, 4, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::cyan, 5, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::blue, 6, true);
  setLEDInBothEyesToColor(ledRequest, BehaviorLEDRequest::green, 7, true);
  rotationState++;
}

void LEDHandler::setRotatingEyesTwoColors(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, int duration, bool mirrored)
{
  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;

  std::vector<bool> firstColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};

  if (rotationState < duration)
  {
    firstColorLEDs = {1, 0, 1, 0, 1, 0, 1, 0};
    rotationState++;
  }
  else if (rotationState < 2 * duration)
  {
    firstColorLEDs = {0, 1, 0, 1, 0, 1, 0, 1};
    rotationState = rotationState == 2 * duration - 1 ? 0 : rotationState + 1;
  }

  for (int i = 0; i < numOfLEDsPerColor; i++)
  {
    if (firstColorLEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col1, i, mirrored);
    }
    else
    {
      setLEDInBothEyesToColor(ledRequest, col2, i, mirrored);
    }
  }
}

void LEDHandler::setRotatingEyesThreeColors(
    LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor transition, int duration, bool mirrored)
{
  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;

  std::vector<bool> firstColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<bool> secondColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<bool> transitionColorLEDs = {0, 0, 0, 0, 0, 0, 0, 0};

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
    rotationState = rotationState == 4 * duration - 1 ? 0 : rotationState + 1;
  }

  for (int i = 0; i < numOfLEDsPerColor; i++)
  {
    if (firstColorLEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col1, i, mirrored);
    }
    else if (secondColorLEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, col2, i, mirrored);
    }
    else if (transitionColorLEDs[i])
    {
      setLEDInBothEyesToColor(ledRequest, transition, i, mirrored);
    }
  }
}

void LEDHandler::setLEDInBothEyesToColor(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col, int led, bool mirrored)
{
  if (led < 0 || led > 7)
    return;

  LEDRequest::LED firstLeft = LEDRequest::faceLeftRed0Deg;
  LEDRequest::LED firstRight = LEDRequest::faceRightRed0Deg;

  static const int redOffsetLeft = 0;
  static const int greenOffsetLeft = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg;
  static const int blueOffsetLeft = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg;

  static const int redOffsetRight = 0;
  static const int greenOffsetRight = LEDRequest::faceRightGreen0Deg - LEDRequest::faceRightRed0Deg;
  static const int blueOffsetRight = LEDRequest::faceRightBlue0Deg - LEDRequest::faceRightRed0Deg;

  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;

  switch (col)
  {
  case BehaviorLEDRequest::defaultColor:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::darkred:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::red:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::green:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::darkgreen:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::blue:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::on;
    break;
  case BehaviorLEDRequest::white:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::on;
    break;
  case BehaviorLEDRequest::grey:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::half;
    break;
  case BehaviorLEDRequest::magenta:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::on;
    break;
  case BehaviorLEDRequest::violet:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::on;
    break;
  case BehaviorLEDRequest::darkyellow:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::yellow:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  case BehaviorLEDRequest::cyan:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::on;
    break;
  case BehaviorLEDRequest::darkblue:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::off;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::half;
    break;
  case BehaviorLEDRequest::orange:
    ledRequest.ledStates[firstLeft + redOffsetLeft + led] = LEDRequest::on;
    ledRequest.ledStates[firstLeft + greenOffsetLeft + led] = LEDRequest::half;
    ledRequest.ledStates[firstLeft + blueOffsetLeft + led] = LEDRequest::off;
    break;
  default:
    ASSERT(false);
    break;
  }

  if (mirrored)
  {
    if (led == 0)
    {
      ledRequest.ledStates[firstRight + redOffsetRight + numOfLEDsPerColor - 8] = ledRequest.ledStates[firstLeft + redOffsetLeft + led];
      ledRequest.ledStates[firstRight + greenOffsetRight + numOfLEDsPerColor - 8] = ledRequest.ledStates[firstLeft + greenOffsetLeft + led];
      ledRequest.ledStates[firstRight + blueOffsetRight + numOfLEDsPerColor - 8] = ledRequest.ledStates[firstLeft + blueOffsetLeft + led];
    }
    else
    {
      ledRequest.ledStates[firstRight + redOffsetRight + numOfLEDsPerColor - led] = ledRequest.ledStates[firstLeft + redOffsetLeft + led];
      ledRequest.ledStates[firstRight + greenOffsetRight + numOfLEDsPerColor - led] = ledRequest.ledStates[firstLeft + greenOffsetLeft + led];
      ledRequest.ledStates[firstRight + blueOffsetRight + numOfLEDsPerColor - led] = ledRequest.ledStates[firstLeft + blueOffsetLeft + led];
    }
  }
  else
  {
    ledRequest.ledStates[firstRight + redOffsetRight + led] = ledRequest.ledStates[firstLeft + redOffsetLeft + led];
    ledRequest.ledStates[firstRight + greenOffsetRight + led] = ledRequest.ledStates[firstLeft + greenOffsetLeft + led];
    ledRequest.ledStates[firstRight + blueOffsetRight + led] = ledRequest.ledStates[firstLeft + blueOffsetLeft + led];
  }
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
  LEDRequest::LEDState gcState = theGameInfo.controllerConnected ? LEDRequest::blinking : LEDRequest::on;

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
  LEDRequest::LEDState state = theBehaviorData.playerNumberToBall == theRobotInfo.number ? LEDRequest::fastBlinking : LEDRequest::on;

  setEyeColor(ledRequest, false, BehaviorLEDRequest::white, LEDRequest::off);

  switch (theRoleSymbols.role)
  {
  case BehaviorData::keeper:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
    break;
  case BehaviorData::defenderLeft:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state, true);
    break;
  case BehaviorData::defenderRight:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state, false, true);
    break;
  case BehaviorData::backupBallchaser:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, state);
    break;
  case BehaviorData::defenderSingle:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state);
    break;
  case BehaviorData::center:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
    break;
  case BehaviorData::replacementKeeper:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::cyan, state);
    break;
  case BehaviorData::receiver:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, state);
    break;
  case BehaviorData::leftWing:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::orange, state, true);
    break;
  case BehaviorData::rightWing:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::orange, state, false, true);
    break;
  case BehaviorData::frontWing:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
    break;
  case BehaviorData::backWing:
    setEyeColor(ledRequest, false, BehaviorLEDRequest::yellow, state);
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
    setEyeColor(ledRequest, false, BehaviorLEDRequest::white, LEDRequest::off);
    setEyeColor(ledRequest, true, BehaviorLEDRequest::white, LEDRequest::off);
    setEyeColor(ledRequest, false, BehaviorLEDRequest::orange, LEDRequest::on);
    setEyeColor(ledRequest, true, BehaviorLEDRequest::orange, LEDRequest::on);
  }
}

/** Set feedback for broken Joints to eyes. */
void LEDHandler::setDamagedJoints(LEDRequest& ledRequest)
{
  if (theBehaviorData.soccerState == BehaviorData::safetyShutdown)
  {
    setRandomizedFireEyes(ledRequest, BehaviorLEDRequest::darkblue, BehaviorLEDRequest::violet, BehaviorLEDRequest::magenta);
  }
}

/** Set game state info to chest button. */
void LEDHandler::setGameStateInfo(LEDRequest& ledRequest)
{
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
    setStaticFireEyes(ledRequest, BehaviorLEDRequest::darkred, BehaviorLEDRequest::orange, BehaviorLEDRequest::yellow);
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

  LEDRequest::LEDState halfState = LEDRequest::off;
  if (s == LEDRequest::on)
    halfState = LEDRequest::half;
  else if (s == LEDRequest::blinking)
    halfState = LEDRequest::halfBlinking;
  else if (s == LEDRequest::fastBlinking)
    halfState = LEDRequest::halfFastBlinking;

  switch (col)
  {
  case BehaviorLEDRequest::defaultColor:
    ASSERT(false);
    break;
  case BehaviorLEDRequest::red:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::darkred:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    break;
  case BehaviorLEDRequest::green:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::darkgreen:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
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
  case BehaviorLEDRequest::grey:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    break;
  case BehaviorLEDRequest::magenta:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::violet:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::darkyellow:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    break;
  case BehaviorLEDRequest::yellow:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::cyan:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    break;
  case BehaviorLEDRequest::darkblue:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + blueOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
    break;
  case BehaviorLEDRequest::orange:
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + redOffset + i] = useLED[i] ? s : LEDRequest::blinking;
    for (int i = 0; i < numOfLEDsPerColor; i++)
      ledRequest.ledStates[first + greenOffset + i] = useLED[i] ? halfState : LEDRequest::halfBlinking;
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
    case WalkRequest::StepRequest::rotateKick45: // CYAN
    case WalkRequest::StepRequest::keeperKick45: // CYAN
      ledRequest.ledStates[r] = LEDRequest::off;
      ledRequest.ledStates[g] = LEDRequest::on;
      ledRequest.ledStates[b] = LEDRequest::on;
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
