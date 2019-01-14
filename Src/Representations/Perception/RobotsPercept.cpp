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

    /*Vector2i startpoint;
    Vector2i endpoint;
    switch (robotEstimate.direction)
    {
    case RobotEstimate::front:
      startpoint.x() = robotEstimate.imageUpperLeft.x() + (robotEstimate.imageLowerRight.x() - robotEstimate.imageUpperLeft.x()) / 2;
      startpoint.y() = robotEstimate.imageUpperLeft.y();
      endpoint.x() = startpoint.x();
      endpoint.y() = robotEstimate.imageLowerRight.y();
      break;
    case RobotEstimate::left:
      startpoint.x() = robotEstimate.imageLowerRight.x();
      startpoint.y() = robotEstimate.imageUpperLeft.y() + (robotEstimate.imageLowerRight.y() - robotEstimate.imageUpperLeft.y()) / 2;
      endpoint.x() = robotEstimate.imageUpperLeft.x();
      endpoint.y() = startpoint.y();
      break;
    case RobotEstimate::right:
      startpoint.x() = robotEstimate.imageUpperLeft.x();
      startpoint.y() = robotEstimate.imageUpperLeft.y() + (robotEstimate.imageLowerRight.y() - robotEstimate.imageUpperLeft.y()) / 2;
      endpoint.x() = robotEstimate.imageLowerRight.x();
      endpoint.y() = startpoint.y();
      break;
    case RobotEstimate::back:
      startpoint.x() = robotEstimate.imageUpperLeft.x() + (robotEstimate.imageLowerRight.x() - robotEstimate.imageUpperLeft.x()) / 2;
      startpoint.y() = robotEstimate.imageLowerRight.y();
      endpoint.x() = startpoint.x();
      endpoint.y() = robotEstimate.imageUpperLeft.y();
      break;
    case RobotEstimate::lying:
      startpoint.x() = robotEstimate.imageUpperLeft.x();
      startpoint.y() = robotEstimate.imageLowerRight.y();
      endpoint.x() = robotEstimate.imageLowerRight.x();
      endpoint.y() = startpoint.y();
      break;
    }*/

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
      /*if (robotEstimate.direction != RobotEstimate::unknown)
        ARROW("representation:RobotsPercept:Image:Lower", 
        startpoint.x(), startpoint.y(), 
        endpoint.x(), endpoint.y(), 
        5, Drawings::solidPen, ColorRGBA::yellow);
      DRAWTEXT("representation:RobotsPercept:Image:Lower", robotEstimate.imageUpperLeft.x(), robotEstimate.imageLowerRight.y() + 15, 10, ColorRGBA::white, robotEstimate.validity << "%");*/
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
      /*if (robotEstimate.direction != RobotEstimate::unknown)
        ARROW("representation:RobotsPercept:Image:Upper",
          startpoint.x(), startpoint.y(),
          endpoint.x(), endpoint.y(),
          5, Drawings::solidPen, ColorRGBA::yellow);
      DRAWTEXT("representation:RobotsPercept:Image:Upper", robotEstimate.imageUpperLeft.x(), robotEstimate.imageLowerRight.y() + 15, 10, ColorRGBA::white, robotEstimate.validity << "%");*/
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

void RobotsHypotheses::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotsHypotheses:Upper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:RobotsHypotheses:Lower", "drawingOnImage");

  for (auto const& hypothesis : robotsHypotheses)
  {
    ColorRGBA color = ColorRGBA::cyan;
    switch (hypothesis.direction) 
    {
      case RobotHypothesis::front:
        color = ColorRGBA::blue; break;
      case RobotHypothesis::left:
        color = ColorRGBA::yellow; break;
      case RobotHypothesis::right:
        color = ColorRGBA::magenta; break;
      case RobotHypothesis::back:
        color = ColorRGBA::white; break;
      case RobotHypothesis::lying:
        color = ColorRGBA::gray; break;
    }
    RECTANGLE("representation:RobotsHypotheses:Lower", hypothesis.upperLeftCorner.x(), hypothesis.upperLeftCorner.y(), hypothesis.lowerRightCorner.x(), hypothesis.lowerRightCorner.y(), 3, Drawings::solidPen, color);
  }

  for (auto const& hypothesis : robotsHypothesesUpper)
  { 
    ColorRGBA color = ColorRGBA::cyan;
    switch (hypothesis.direction)
    {
      case RobotHypothesis::front:
        color = ColorRGBA::blue; break;
      case RobotHypothesis::left:
        color = ColorRGBA::yellow; break;
      case RobotHypothesis::right:
        color = ColorRGBA::magenta; break;
      case RobotHypothesis::back:
        color = ColorRGBA::white; break;
      case RobotHypothesis::lying:
        color = ColorRGBA::gray; break;
    }
    RECTANGLE("representation:RobotsHypotheses:Upper", hypothesis.upperLeftCorner.x(), hypothesis.upperLeftCorner.y(), hypothesis.lowerRightCorner.x(), hypothesis.lowerRightCorner.y(), 3, Drawings::solidPen, color);
  }

}
