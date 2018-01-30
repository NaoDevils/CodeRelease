/**
* \file BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* The ball member variables are the ones the robot's decisions are based on.
* \author Ingmar Schwarz
*/ 

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"

/**
* \class BehaviorData
* A class that containts data about the current behavior state.
*/ 
STREAMABLE(BehaviorData,
{
  ENUM(RoleAssignment,
  {,
    undefined,
    firstRole,
    keeper = firstRole,
    defender,
    supporterDef,
    striker,
    supporterOff,
  });
  
  ENUM(Action,
  { ,
    inactive,
    penalized,
    nothing,
    lost,
    numOfUnsafeActions,
    pass = numOfUnsafeActions,
    controlBall,
    positioning,
    searchForBall,
    standUp,
  });

  BehaviorData& operator=(const BehaviorData &other)
  {
    if (this == &other)
      return *this;
    role = other.role;
    soccerState = other.soccerState;
    lastRole = other.lastRole;
    ballPositionRelative = other.ballPositionRelative;
    ballPositionField = other.ballPositionField;
    ballPositionFieldPredicted = other.ballPositionFieldPredicted;
    timeSinceBallWasSeen = other.timeSinceBallWasSeen;
    return *this;
  },
  (Action)(nothing) soccerState, /**< What is the robot doing in general? */
  (RoleAssignment)(undefined) role, /**< A dynamically chosen role. */
  (RoleAssignment)(undefined) lastRole, /**< For role sync. */
  (Vector2s)(Vector2s::Zero()) ballPositionRelative, /**< The robot's relative ball position w/preview. */
  (Vector2s)(Vector2s::Zero()) ballPositionField, /**< The robot's ball position on field. */
  (Vector2s)(Vector2s::Zero()) ballPositionFieldPredicted, /**< The robot's predicted ball position on field. */
  (int)(1000000) timeSinceBallWasSeen, /**< Time since the robot last saw the ball himself. */
});

STREAMABLE(BehaviorControlOutput,
{,
  (BehaviorData) behaviorData,
});