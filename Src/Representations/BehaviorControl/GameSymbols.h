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
STREAMABLE(GameSymbols,,
  (bool)(false) avoidCenterCircle,
  (bool)(false) allowedInGoalArea,
  (bool)(false) kickoffInProgress,
  (bool)(false) ownKickOff,
  (bool)(false) ballKickedOutOfCenterCircle,
  (uint8_t)(0) currentSetPlay, // the set play currently in progress
  (uint8_t)(0) lastSetPlay, // the last set play
  (int)(0) timeSinceSetPlayStarted, // time (in seconds) since current set play started
  (int)(1000) timeSinceSetPlayFinished, // time (in seconds) since last set play finished
  (int)(-1) timeSinceLastBallOut, // time (in seconds) when the ball last left the field (-1 before first ball out)
  (int)(-1) lastBallOutTeam, // number of team that caused last ball out (-1 before first ball out)
  (int)(100000) timeSincePlayingState,
  (int)(100000) timeSinceLastPenalty,
  (uint8_t)(0) lastGameState,  (int)(0) timeSinceGameState);
