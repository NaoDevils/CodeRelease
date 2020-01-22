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
  for(int i = 0; i < ledRequest.numOfLEDs; ++i)
    ledRequest.ledStates[i] = LEDRequest::off;

  //update
  setRightEar(ledRequest);
  setLeftEar(ledRequest);
  setRightEye(ledRequest);
  setLeftEye(ledRequest);
  setChestButton(ledRequest);
  setHead(ledRequest);
  setFeet(ledRequest);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest)
{
  //right ear -> battery
  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar];

  int onLEDs = std::min(static_cast<int>(theSystemSensorData.batteryLevel / 0.1f), 9);

  for(int i = 0; i <= onLEDs; i++)
    ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
  //left ear -> connected players
  //          + GameController connection lost -> freaky blinking
  if(theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > 2000)
  {
    ledRequest.ledStates[LEDRequest::earsLeft324Deg] = LEDRequest::blinking;
    ledRequest.ledStates[LEDRequest::earsLeft144Deg] = LEDRequest::blinking;
  }

  int numberOfConnectedTeammates = static_cast<int>(theTeammateData.teammates.size());
  if(numberOfConnectedTeammates > 0)
  {
    ledRequest.ledStates[LEDRequest::earsLeft0Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft36Deg] = LEDRequest::on;
  }
  if(numberOfConnectedTeammates > 1)
  {
    ledRequest.ledStates[LEDRequest::earsLeft72Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft108Deg] = LEDRequest::on;
  }
  if(numberOfConnectedTeammates > 2)
  {
    ledRequest.ledStates[LEDRequest::earsLeft180Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft216Deg] = LEDRequest::on;
  }
  if(numberOfConnectedTeammates > 3)
  {
    ledRequest.ledStates[LEDRequest::earsLeft252Deg] = LEDRequest::on;
    ledRequest.ledStates[LEDRequest::earsLeft288Deg] = LEDRequest::on;
  }

  // in set override ears for battery with ears for whistle detection
  LEDRequest::LEDState state;
  if (theGameInfo.state == STATE_SET || theGameSymbols.timeSincePlayingState < 10000)
  {
    if (theWhistleDortmund.detectionState == WhistleDortmund::DetectionState::isDetected)
      state = LEDRequest::LEDState::on;
    else
      state = LEDRequest::LEDState::off;
    for (int i = 0; i <= 9; i++)
      ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
  }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest,
                             bool left,
                             BehaviorLEDRequest::EyeColor col,
                             LEDRequest::LEDState s,
                             bool onlyLeft,
                             bool onlyRight)
{
  LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

  static const int redOffset = 0,
    greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
    blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg;

  static const int numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg + 1;
  std::vector<bool> useLED = { 1,1,1,1,1,1,1,1 };
  if (onlyLeft) useLED = { 1,1,1,1,1,0,0,0 };
  if (onlyRight) useLED = { 1,0,0,0,1,1,1,1 };

  LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

  switch(col)
  {
    case BehaviorLEDRequest::defaultColor:
      ASSERT(false);
      break;
    case BehaviorLEDRequest::red:
      for(int i = 0; i < numOfLEDsPerColor; i++)
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
    default:
      ASSERT(false);
      break;
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest)
{
  //left eye -> groundContact ? ballSeen and GoalSeen : blue
  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];

  //no groundContact
  if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::defaultColor)
    //blue
    setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
  //default
  else
  {
    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
    bool goalSeen = theFrameInfo.getTimeSince(theCLIPGoalPercept.timeWhenLastSeen) < 250;

    if(ballSeen && goalSeen)
      //red
      setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    else if(ballSeen)
      //white
      setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
    else if(goalSeen)
      //green
      setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest)
{
  //right eye -> groundContact ? role : role -> blinking
  //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off
  LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

  //no groundContact
  if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
  //overwrite
  else if(theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::defaultColor)
    setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);
  else
  {
    switch(theRoleSymbols.role)
    {
      case BehaviorData::keeper:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
        break;
      case BehaviorData::undefined:
        //off
        break;
      default:
        ASSERT(false);
    }
  }
}

void LEDHandler::setHead(LEDRequest& ledRequest)
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

  // in set override ears for battery with ears for whistle detection
  if (theGameInfo.state == STATE_SET || theGameSymbols.timeSincePlayingState < 10000)
  {
    LEDRequest::LEDState state;
    if (theWhistleDortmund.detectionState == WhistleDortmund::DetectionState::isDetected)
      state = LEDRequest::LEDState::on;
    else
      state = LEDRequest::LEDState::off;

    for (unsigned i = LEDRequest::headLedRearLeft0; i <= LEDRequest::headLedMiddleLeft0; i++)
      ledRequest.ledStates[i] = state;
  }
}

void LEDHandler::setChestButton(LEDRequest& ledRequest)
{
  for (int i = 0; i < 3; i++) ledRequest.ledStates[LEDRequest::chestRed + i] = LEDRequest::off;
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

void LEDHandler::setFeet(LEDRequest& ledRequest)
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
}

MAKE_MODULE(LEDHandler, behaviorControl)
