/**
* \file GameSymbols.h
* The file declares a class that containts data about the current behavior state.
* \author Oliver Urbann
*/ 

#pragma once
#include "Tools/Streams/AutoStreamable.h"

/**
* \class GameSymbols
* A class that containts data about the current behavior state.
*/ 
STREAMABLE(GameSymbols,
{,
  (bool)(false) avoidCenterCircle,
  (bool)(false) allowedInPenaltyArea,
  (bool)(false) kickoffInProgress,
  (bool)(false) ownKickOff,
  (int)(100000) timeSincePlayingState,
  (int)(100000) timeSinceLastPenalty,
  (uint8_t)(0) lastGameState,  (int)(0) timeSinceGameState,});
