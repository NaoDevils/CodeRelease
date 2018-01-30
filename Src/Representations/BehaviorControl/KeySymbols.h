/**
* \file KeySymbols.h
* The file declares a class that containts data about the current behavior state.
* \author Oliver Urbann
*/ 

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

/**
* \class KeySymbols
* A class that containts data about the current behavior state.
*/ 
class KeySymbols : public Streamable
{
private:

  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN;
    STREAM(obstacle_hit);
    STREAM(timeLeftFootPressed);
    STREAM(timeRightFootPressed);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
  * Default constructor.
  */
  KeySymbols() : 
      obstacle_hit(false),
      timeLeftFootPressed(0),
      timeRightFootPressed(0)
  {
    for (int i = 0; i < KeyStates::numOfKeys; i++)
      pressed_and_released[i] = false;
  }
  
  bool obstacle_hit;
  int timeLeftFootPressed;
  int timeRightFootPressed;
  bool pressed_and_released[KeyStates::numOfKeys];
};
