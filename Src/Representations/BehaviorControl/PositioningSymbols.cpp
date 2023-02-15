#include "PositioningSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void PositioningSymbols::draw() const
{
  // positioning symbol drawings in blue
  DEBUG_DRAWING("representation:PositioningSymbols", "drawingOnField")
  {
    POSE_2D_SAMPLE("representation:PositioningSymbols", optPosition, ColorRGBA::blue);
    ELLIPSE("representation:PositioningSymbols", optPosition.translation, (thresholdXBack + thresholdXFront) / 2, thresholdY, optPosition.rotation, 5, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
  }
}