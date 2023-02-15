/**
* \file KeySymbols.h
* The file declares a class that containts data about the current behavior state.
* \author Oliver Urbann
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

/**
* \class KeySymbols
* A class that containts data about the current behavior state.
*/
STREAMABLE(KeySymbols,
  /**
  * Default constructor.
  */
  KeySymbols()
  {
    for (int i = 0; i < KeyStates::numOfKeys; i++)
    {
      pressed_and_released[i] = false;
      lastTimeNotPressed[i] = 0;
    }
  }
  ,
  (bool)(false) obstacle_hit,
  (int)(0) timeLeftFootPressed,
  (int)(0) timeRightFootPressed,
  (bool[KeyStates::numOfKeys]) pressed_and_released,
  (unsigned[KeyStates::numOfKeys]) lastTimeNotPressed
);
