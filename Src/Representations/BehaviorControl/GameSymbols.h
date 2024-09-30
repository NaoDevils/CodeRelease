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
  ENUM(GameSituation,
      none, // initial, standby, timeout, finished, ...

      kickOff_own_ready,
      kickOff_own_set,
      kickOff_own_playing_ballNotFree,

      kickOff_opponent_ready,
      kickOff_opponent_set,
      kickOff_opponent_playing_ballNotFree,

      goalKick_own,
      goalKick_opponent,

      pushingFreeKick_own,
      pushingFreeKick_opponent,

      cornerKick_own,
      cornerKick_opponent,

      kickIn_own,
      kickIn_opponent,

      penaltyKick_own_ready,
      penaltyKick_own_set,
      penaltyKick_own_playing,

      penaltyKick_opponent_ready,
      penaltyKick_opponent_set,
      penaltyKick_opponent_playing,

      regularPlay
  )
  ,
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
  (int)(1) lastKickTime,
  (bool)(false) kickedThisFrame,
  (uint8_t)(0) lastGameState,
  (int)(0) timeSinceGameState,
  (GameSituation) gameSituation
);
