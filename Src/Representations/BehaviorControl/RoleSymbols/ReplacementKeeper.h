/**
* \file ReplacementKeeper.h
* The file declares a class that containts data about the desired position of the replacementKeeper on the field.
*/

#pragma once

#include "PositioningSymbols.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
* \class ReplacementKeeper
* The file declares a class that containts data about the desired position of the replacementKeeper on the field.
*/
STREAMABLE_WITH_BASE(ReplacementKeeper, PositioningSymbols,
  ReplacementKeeper& operator=(const ReplacementKeeper &other)
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
    blockBall = other.blockBall;
    kickIt = other.kickIt;
    return *this;
  }
  /* Drawings, in world coordinate system */
  //void draw() const
  ,
  // special members for this role
  (bool)(false) blockBall,
  (bool)(false) kickIt
);
