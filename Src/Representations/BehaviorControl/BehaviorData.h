/**
* \file BehaviorData.h
* The file declares a class that containts data about the current behavior state.
* \author Colin Graf
*/ 

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

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
    return *this;
  },
  (Action)(nothing) soccerState, /**< What is the robot doing in general? */
  (RoleAssignment)(undefined) role, /**< A dynamically chosen role. */
  (RoleAssignment)(undefined) lastRole, /**< For role sync. */
});

STREAMABLE(BehaviorControlOutput,
{,
  (BehaviorData) behaviorData,
});