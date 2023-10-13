/**
* \file DefenderLeft.h
* The file declares a class that containts data about the desired position of the defenderLeft on the field.
*/

#pragma once
#include "PositioningSymbols.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

/**
* \class DefenderLeft
* The file declares a class that containts data about the desired position of the defenderLeft on the field.
*/
STREAMABLE_WITH_BASE(DefenderLeft, PositioningSymbols,
  /* Drawings, in world coordinate system */
  //void draw() const
  ,
);
