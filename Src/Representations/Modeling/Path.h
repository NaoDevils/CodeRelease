/**
* @file Path.h
* Declaration of a class that represents the path to a robots target.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#ifndef __Path_h_
#define __Path_h_

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <vector>

/**
* @class Path
* A class that represents the path to the robots target.
*/
STREAMABLE(Path,
{
  /** Reset the path */
  void reset()
  {
    wayPoints.clear();
    nearestObstacle = 10000.f;
    distanceToBall = 10000.f;
  };
  /**
  * The method draws the path.
  */
  void draw() const
  {
    //2D
    DEBUG_DRAWING("representation:Path", "drawingOnField")
    {
      // draw next path points
      for (auto const& i : wayPoints)
      {
        POSE_2D_SAMPLE("representation:Path", i, ColorRGBA(255, 0, 0));
      }
    }
    // 3D
    DEBUG_DRAWING3D("representation:Path", "field")
    {
      Pose2f last = *(wayPoints.begin());
      for (std::vector<Pose2f>::const_iterator i = ++wayPoints.begin(); i != wayPoints.end(); ++i)
      {
        LINE3D("representation:Path", last.translation.x(), last.translation.y(), 1.f, i->translation.x(), i->translation.y(), 1.f, 2.f, ColorRGBA::yellow);
        Vector3f from(i->translation.x(), i->translation.y(), 1.f);
        Vector3f to(from + Vector3f(std::cos(i->rotation) * 100, std::sin(i->rotation) * 100, 0.f));
        CYLINDERARROW3D("representation:Path", from, to, 2.f, 30.f, 5.f, ColorRGBA::yellow);
        last = *i;
      }
    }
  },
  (std::vector<Pose2f>) wayPoints, /**< The wayPoints in absolute field coordinates. */
  (float)(10000.f) nearestObstacle,
  (float)(100.f) distanceToBall, /**< The distance to the ball calculated from the path to the ball and the rotation towards the ball */  
});

#endif //__Path_h_
