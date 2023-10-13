/**
* \file BallChaserDecision.h
* The file declares a class that contains the player number who is best suited to go for the ball.
* \author Ingmar Schwarz
*/

#pragma once
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BallChaserDecision,
  void draw() const;
  ,
  (int)(1) playerNumberToBall, // decision from lowest player number
  (float)(100000.f) ownTimeToBall,
  (bool)(true) dynamic
);
