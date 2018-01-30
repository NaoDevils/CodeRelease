/**
* \file BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* \author Oliver Urbann
*/ 

#pragma once
#include "Tools/Streams/Streamable.h"

/**
* \class BehaviorData
* A class that containts data about the current behavior state.
*/ 
class TeamSymbols : public Streamable
{
private:

  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN;
    STREAM(howMuchOfOwnGoalCoveredByOtherFieldPlayer);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
  * Default constructor.
  */
  TeamSymbols() : 
      howMuchOfOwnGoalCoveredByOtherFieldPlayer(0)
  {
  }
  
  float howMuchOfOwnGoalCoveredByOtherFieldPlayer;
};
