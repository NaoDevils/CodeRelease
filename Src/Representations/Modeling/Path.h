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
  ENUM(ObstacleType,
  {,
    ball,
    goalPost,
    robot,
    teamRobot,
    centerCircle,
    setPlayCircle,
  });
  /** Reset the path */
  void reset()
  {
    wayPoints.clear();
    nearestObstacle = std::numeric_limits<float>::max();

    length = 0.f;
    useExtraBallPoint = false;
  };
  /**
  * The method draws the path.
  */
  void draw() const
  {
    //2D
    DEBUG_DRAWING("representation:Path", "drawingOnField")
    {
      const Pose2f* prev = nullptr;
      for (std::vector<Pose2f>::const_iterator wpIt = wayPoints.cbegin(); wpIt != wayPoints.cend(); ++wpIt) {
        Vector2f direction = Vector2f(300, 0);
        direction.rotate(wpIt->rotation);
        direction += wpIt->translation;
        ARROW(
          "representation:Path",
          wpIt->translation.x(),
          wpIt->translation.y(),
          direction.x(),
          direction.y(),
          8,
          Drawings::solidPen,
          ColorRGBA::red
        );

        if (prev) {
          LINE("representation:Path", prev->translation.x(), prev->translation.y(), wpIt->translation.x(), wpIt->translation.y(), 8, Drawings::solidPen, ColorRGBA::white);
        }

        prev = &*wpIt;
      }
    }
    // 3D
    DEBUG_DRAWING3D("representation:Path", "field")
    {
      Pose2f last = *(wayPoints.begin());
      for (std::vector<Pose2f>::const_iterator i = ++wayPoints.begin(); i != wayPoints.end(); ++i)
      {
        if (useExtraBallPoint)
          LINE3D("representation:Path", last.translation.x(), last.translation.y(), 1.f, i->translation.x(), i->translation.y(), 1.f, 2.f, ColorRGBA::red);
        else
          LINE3D("representation:Path", last.translation.x(), last.translation.y(), 1.f, i->translation.x(), i->translation.y(), 1.f, 2.f, ColorRGBA::yellow);
        
        Vector3f from(i->translation.x(), i->translation.y(), 1.f);
        Vector3f to(from + Vector3f(std::cos(i->rotation) * 100, std::sin(i->rotation) * 100, 0.f));
        CYLINDERARROW3D("representation:Path", from, to, 2.f, 30.f, 5.f, ColorRGBA::yellow);
        last = *i;
      }
    }
  },
    
  (bool)(false) useExtraBallPoint,
  (std::vector<Pose2f>) wayPoints, /**< The wayPoints in absolute field coordinates. */
  (float)(10000.f) nearestObstacle,
  (Vector2f)(Vector2f::Zero()) nearestObstaclePosition,
  (ObstacleType)(robot) nearestObstacleType,
  (float)(0.f) length, /**< The distance to the target calculated from the path. */  
});

#endif //__Path_h_
