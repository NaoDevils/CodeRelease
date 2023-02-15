/** 
* @file KeySymbolsProvider.cpp
*
* Implementation of class KeySymbolsProvider.
*/

#include "KeySymbolsProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"

void KeySymbolsProvider::update(KeySymbols& keySymbols)
{
  for (int i = 0; i < KeyStates::numOfKeys; i++)
  {
    if (!theKeyStates.pressed[i])
      keySymbols.lastTimeNotPressed[i] = theFrameInfo.time;
    else if (theFrameInfo.getTimeSince(keySymbols.lastTimeNotPressed[i]) > 200)
      last_pressed_time[i] = theFrameInfo.time;
    if (theFrameInfo.getTimeSince(last_pressed_time[i]) > 0 && theFrameInfo.getTimeSince(last_pressed_time[i]) < 400)
      keySymbols.pressed_and_released[i] = true;
    else
      keySymbols.pressed_and_released[i] = false;
  }

  keySymbols.timeLeftFootPressed = 0;
  keySymbols.timeRightFootPressed = 0;
  if (keySymbols.lastTimeNotPressed[KeyStates::leftFootLeft] == theFrameInfo.time && keySymbols.lastTimeNotPressed[KeyStates::leftFootLeft] > last_pressed_time[KeyStates::leftFootLeft])
    keySymbols.timeLeftFootPressed = keySymbols.lastTimeNotPressed[KeyStates::leftFootLeft] - last_pressed_time[KeyStates::leftFootLeft];
  if (keySymbols.lastTimeNotPressed[KeyStates::leftFootRight] == theFrameInfo.time && keySymbols.lastTimeNotPressed[KeyStates::leftFootRight] > last_pressed_time[KeyStates::leftFootRight])
    keySymbols.timeLeftFootPressed = std::max<int>(keySymbols.timeLeftFootPressed, keySymbols.lastTimeNotPressed[KeyStates::leftFootRight] - last_pressed_time[KeyStates::leftFootRight]);
  if (keySymbols.lastTimeNotPressed[KeyStates::rightFootLeft] == theFrameInfo.time && keySymbols.lastTimeNotPressed[KeyStates::rightFootLeft] > last_pressed_time[KeyStates::rightFootLeft])
    keySymbols.timeRightFootPressed = keySymbols.lastTimeNotPressed[KeyStates::rightFootLeft] - last_pressed_time[KeyStates::rightFootLeft];
  if (keySymbols.lastTimeNotPressed[KeyStates::rightFootRight] == theFrameInfo.time && keySymbols.lastTimeNotPressed[KeyStates::rightFootRight] > last_pressed_time[KeyStates::rightFootRight])
    keySymbols.timeRightFootPressed = std::max<int>(keySymbols.timeLeftFootPressed, keySymbols.lastTimeNotPressed[KeyStates::rightFootRight] - last_pressed_time[KeyStates::rightFootRight]);

  // HACK!
  // the need to press the button for a bit longer
  // confuses the referees, so map "pressed[KeyStates::chest]" directly to "pressed_and_released[KeyStates::chest]"
  keySymbols.pressed_and_released[KeyStates::chest] = theKeyStates.pressed[KeyStates::chest];

  // Security check for broken bumpers
  for (int i = 0; i < 4; i++)
  {
    buttonCheckFilter[i].push_front(theKeyStates.pressed[KeyStates::leftFootLeft + i]);
  }


  // Calculate if there is an obstacle
  int pressSum = 0;
  for (int i = KeyStates::leftFootLeft; i < KeyStates::chest; i++)
    if (buttonCheckFilter[i - KeyStates::leftFootLeft].average() < 0.5) // use only if not broken
      pressSum += theKeyStates.pressed[i];

  pressBuffer.push_front((float)pressSum);
  PLOT("module:KeySymbols:bufferAvg", pressBuffer.average());
  //PLOT("module:KeySymbols:RBumpR",theKeyStates.pressed[KeyStates::rightFootRight]);
  //PLOT("module:KeySymbols:RBumpL",theKeyStates.pressed[KeyStates::rightFootLeft]);
  //PLOT("module:KeySymbols:LBumpR",theKeyStates.pressed[KeyStates::leftFootRight]);
  //PLOT("module:KeySymbols:LBumpL",theKeyStates.pressed[KeyStates::leftFootLeft]);
  //PLOT("module:KeySymbols:ChestButton",theKeyStates.pressed[KeyStates::chest]);

  bool obstacle_hit = pressBuffer.average() > minBufferAvgForObstacle;

  if (obstacle_hit)
    timeWhenObstacleHit = theFrameInfo.time;
  if (theFrameInfo.getTimeSince(timeWhenObstacleHit) < 800 && timeWhenObstacleHit > 0)
    keySymbols.obstacle_hit = true;
  else
    keySymbols.obstacle_hit = false;
}

MAKE_MODULE(KeySymbolsProvider, behaviorControl)
