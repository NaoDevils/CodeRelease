#include "FrameInfo.h"
#include "Tools/Debugging/DebugDrawings.h"

void FrameInfo::draw() const
{
  DEBUG_DRAWING("representation:FrameInfo", "drawingOnField")
  {
    const unsigned seconds = time / 1000;
    const unsigned minutes = seconds / 60;
    const unsigned hours = minutes / 60;
    DRAWTEXT("representation:FrameInfo", -5000, -3000, 200, ColorRGBA::white, "System time:");

    if (cycleTime == 0.012f) // motion cycle
      DRAWTEXT("representation:FrameInfo", -3300, -3000, 200, ColorRGBA::white, "[m] " << hours << ":" << (minutes % 60) << ":" << (seconds % 60) << "." << (time % 1000));
    else
      DRAWTEXT("representation:FrameInfo", -1200, -3000, 200, ColorRGBA::white, "[c] " << hours << ":" << (minutes % 60) << ":" << (seconds % 60) << "." << (time % 1000));
  }
}
