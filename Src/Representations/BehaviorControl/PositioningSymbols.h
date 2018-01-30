/**
* \file PositioningSymbols.h
* The file declares a class that containts data about the desired robot's position on the field.
*/ 

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/BehaviorData.h"

/**
* \class PositioningSymbols
* A class that containts data about the desired robot's position on the field.
*/ 
STREAMABLE(PositioningSymbols,
{
  PositioningSymbols& operator=(const PositioningSymbols &other)
  {
    if (this == &other)
      return *this;
    optPosition = other.optPosition;
    stopAtTarget = other.stopAtTarget;
    previewArrival = other.previewArrival;
    thresholdXFront = other.thresholdXFront;
    thresholdXBack = other.thresholdXBack;
    thresholdY = other.thresholdY;
    thresholdRotation = other.thresholdRotation;
    interceptBall = other.interceptBall;
    ballChaserNo = other.ballChaserNo;
    return *this;
  }
  /* Drawings, in world coordinate system */
  void draw() const,
  (Pose2f)(Pose2f()) optPosition,
  (bool)(false) stopAtTarget,
  (bool)(false) previewArrival,
  (float)(30.f) thresholdXFront, 
  (float)(30.f) thresholdXBack,
  (float)(30.f) thresholdY,
  (Angle)(10_deg) thresholdRotation,
  (bool)(false) interceptBall,
  
  (unsigned)(1) numberOfActiveFieldPlayers, // myself and active teammates excluding keeper
  (Pose2f) ballChaserPosition,
  ((BehaviorData) RoleAssignment)(striker) ballChaserRole,
  (int)(4) ballChaserNo,
  (bool)(false) iAmSupported,
});
