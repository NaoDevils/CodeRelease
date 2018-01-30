#include "RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void ObstacleBasePoints::draw() const
{
  DEBUG_DRAWING("representation:ObstacleBasePoints:Image:Lower", "drawingOnImage")
  {
    for (auto const& obp : basePoints)
    {
      if (!obp.upperCam)
      {
        ColorRGBA color = (obp.direction == ObstacleBasePoint::up) ? ColorRGBA::orange :
          ((obp.direction == ObstacleBasePoint::down) ? ColorRGBA::yellow :
            ((obp.direction == ObstacleBasePoint::left) ? ColorRGBA::magenta : ColorRGBA(80, 200, 200)));
        CIRCLE("representation:ObstacleBasePoints:Image:Lower",
          obp.pointInImage.x(), obp.pointInImage.y(), 10,
          3, Drawings::solidPen, color,
          Drawings::noBrush, ColorRGBA::gray);
      }
    }
  }

  DEBUG_DRAWING("representation:ObstacleBasePoints:Image:Upper", "drawingOnImage")
  {
    for (auto const& obp : basePoints)
    {
      if (obp.upperCam)
      {
        ColorRGBA color = (obp.direction == ObstacleBasePoint::up) ? ColorRGBA::orange :
          ((obp.direction == ObstacleBasePoint::down) ? ColorRGBA::yellow :
            ((obp.direction == ObstacleBasePoint::left) ? ColorRGBA::magenta : ColorRGBA(80, 200, 200)));
        CIRCLE("representation:ObstacleBasePoints:Image:Upper",
          obp.pointInImage.x(), obp.pointInImage.y(), 10,
          3, Drawings::solidPen, color,
          Drawings::noBrush, ColorRGBA::gray);
      }
    }
  }
}

void RobotsPercept::draw() const
{
  //image
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Image:Upper", "drawingOnImage");

  for (auto const& robotEstimate : robots)
  {
    ColorRGBA color = robotEstimate.robotType == RobotEstimate::teammateRobot ?
      ColorRGBA::green : (robotEstimate.robotType == RobotEstimate::opponentRobot ?
        ColorRGBA::red : ColorRGBA::black);
    if (!robotEstimate.fromUpperImage)
    {
      RECTANGLE("representation:RobotsPercept:Image:Lower",
        robotEstimate.imageUpperLeft.x(),
        std::min<int>(1, robotEstimate.imageUpperLeft.y()),
        robotEstimate.imageLowerRight.x(),
        robotEstimate.imageLowerRight.y(),
        5,
        Drawings::solidPen,
        color);
    }
    else
    {
      RECTANGLE("representation:RobotsPercept:Image:Upper",
        robotEstimate.imageUpperLeft.x(),
        robotEstimate.imageUpperLeft.y(),
        robotEstimate.imageLowerRight.x(),
        robotEstimate.imageLowerRight.y(),
        5,
        Drawings::solidPen,
        color);
    }
  }
  //Field

  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Field", "drawingOnField");

  for (auto const& robotEstimate : robots)
  {
    ColorRGBA color = robotEstimate.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green :
                      (robotEstimate.robotType == RobotEstimate::opponentRobot ? ColorRGBA::red :
                      ColorRGBA::black);
    CIRCLE("representation:RobotsPercept:Field",
      robotEstimate.locationOnField.translation.x(), robotEstimate.locationOnField.translation.y(),
      40, 2, Drawings::solidPen, color, Drawings::solidBrush, color);
  }
};
