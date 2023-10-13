/**
* \file Center.h
* The file declares a class that containts data about the desired position of the center on the field.
*/

#pragma once
#include "PositioningSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Streamable.h"

/**
* \class Center
* The file declares a class that containts data about the desired position of the center on the field.
*/
STREAMABLE_WITH_BASE(Center, PositioningSymbols,

  /* Drawings, in world coordinate system */
  //void draw() const
  ,
  (bool)(false) keeperPosKnown,
  (float)(0) keeperPosY,
  (bool)(false) defenderPosKnown,
  (float)(0) defenderPosY,
  (float)(0) destinationY
);
