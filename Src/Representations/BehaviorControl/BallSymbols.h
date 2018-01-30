/**
* \file BallSymbols.h
* The file declares a class that containts data about the current ball state and used ball model.
* \author Ingmar Schwarz
*/ 

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
* \class BallSymbols
* A class that containts data about the current ball state.
*/ 
STREAMABLE(BallSymbols,
{  
  void draw() const,
  (bool)(false) ballWasSeen,
  (bool)(false) ballLost,
  (bool)(false) ballLostForTeam,
  (bool)(false) ballLastSeenLeft,
  (bool)(false) ballFoundAfterDropIn,
  (bool)(false) ballInOwnPenaltyArea,
  (Vector2f)(Vector2f::Zero()) ballPositionRelative,
  (Vector2f)(Vector2f::Zero()) ballPositionRelativeWOPreview,
  (Vector2f)(Vector2f::Zero()) ballVelocityRelative,
  (Vector2f)(Vector2f::Zero()) ballVelocityRelativeWOPreview,
  (Vector2f)(Vector2f::Zero()) ballPositionField,
  (Vector2f)(Vector2f::Zero()) ballPositionFieldPredicted, 
  (bool)(false) ballHitsMe,
  (bool)(false) ballBlockable, // can robot use the block motion to catch the ball?
  (bool)(false) obstacleBlockingBall,
  (float)(0.f) yPosWhenBallReachesOwnYAxis,
  (unsigned int)(100000) timeUntilBallReachesOwnYAxis,
  (int)(100000) timeSinceLastSeen,
  (int)(100000) timeSinceLastSeenByTeam,
  (int)(100000) timeSinceLastSeenByTeamMates,
  (bool)(true) useLocalBallModel,
});
