#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"
#include <vector>

STREAMABLE(SimpleRobotsDistributed,
public:
  STREAMABLE(SimpleRobotDistributed,,
    (short) x, 
    (short) y,
    (short) r, 
    (short) colorCount, 
    (short) validity,
    (Vector2s) velocity,
    (unsigned) timeOfLastUpdate
  );


  void draw() const
  {
    DEBUG_DRAWING("representation:SimpleRobotsDistributed", "drawingOnField")
    {
      for (std::vector<SimpleRobotDistributed>::const_iterator i = robots.begin(); i != robots.end(); ++i)
      {
        ColorRGBA color =
          ColorRGBA(
            (unsigned char)std::min(std::max(0, 128 - 10 * i->colorCount), 255),
            (unsigned char)std::min(std::max(0, 128 + 10 * i->colorCount), 255),
            0);
        CIRCLE("representation:SimpleRobotsDistributed", i->x, i->y, 100, 10,
          Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
}
}

// now in 3D
DEBUG_DRAWING3D("representation:SimpleRobotsDistributed", "field")
{
  for (std::vector<SimpleRobotDistributed>::const_iterator i = robots.begin(); i != robots.end(); ++i)
  {
    ColorRGBA color = ColorRGBA((unsigned char)std::min(std::max(0, 128 - 10 * i->colorCount), 255), (unsigned char)std::min(std::max(0, 128 + 10 * i->colorCount), 255), 0);
    float x = (float)i->x;
    float y = (float)i->y;
    CIRCLE3D("representation:SimpleRobotsDistributed", Pose3f(x, y, 1), 200, 5, color);
  }
}
},
  (std::vector<SimpleRobotDistributed>) robots
);