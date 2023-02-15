/**
* \file DefenderLeft.h
* The file declares a class that containts data about the desired position of the defenderLeft on the field.
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
* \class DefenderLeft
* The file declares a class that containts data about the desired position of the defenderLeft on the field.
*/
STREAMABLE(DefenderLeft,
  DefenderLeft& operator=(const DefenderLeft &other)
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
    return *this;
  }
  /* Drawings, in world coordinate system */
  //void draw() const
  ,

  (Pose2f)(Pose2f()) optPosition,
  (bool)(false) stopAtTarget,
  (bool)(false) previewArrival,
  (float)(30.f) thresholdXFront,
  (float)(30.f) thresholdXBack,
  (float)(30.f) thresholdY,
  (Angle)(10_deg) thresholdRotation
);
