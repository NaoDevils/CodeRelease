#include "KickSymbols.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void KickSymbols::draw() const
{
  // kick symbol drawings in green
  DEBUG_DRAWING("representation:KickSymbols", "drawingOnField")
  {
    POSE_2D_SAMPLE("representation:KickSymbols", kickPosition, ColorRGBA::green);
    ARROW("representation:KickSymbols", 
      kickPosition.translation.x(), kickPosition.translation.y(), kickTarget.x(), kickTarget.y(), 
      5, Drawings::solidPen, ColorRGBA::green);
    ELLIPSE("representation:KickSymbols", kickPosition.translation,
      (thresholdXBack + thresholdXFront) / 2, thresholdY, kickPosition.rotation, 5, Drawings::solidPen, ColorRGBA::green, Drawings::noBrush, ColorRGBA::green);
  }
}