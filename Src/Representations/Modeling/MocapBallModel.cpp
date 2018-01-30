/**
* @file MocapBallModel.cpp
* Implementation of the MocapBallModel's drawing functions
*/

#include "MocapBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"

void MocapBallModel::draw() const
{
  // drawing of the ball model in the field view
  DEBUG_DRAWING("representation:MocapBallModel", "drawingOnField")
  {
    const ColorRGBA transparentRed(255, 0, 0, 128);
    CIRCLE("representation:MocapBallModel",
      translation.x(), translation.y(), 45, 0, // pen width
      Drawings::solidPen, transparentRed,
      Drawings::solidBrush, transparentRed);
  }
}