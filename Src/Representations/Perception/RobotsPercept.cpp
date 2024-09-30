#include "RobotsPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/ImageCoordinateTransformations.h"

#include <iomanip>
#include <sstream>

void ObstacleBasePoints::draw() const
{
  DEBUG_DRAWING("representation:ObstacleBasePoints:Image:Lower", "drawingOnImage")
  {
    for (auto const& obp : basePoints)
    {
      if (!obp.upperCam)
      {
        ColorRGBA color = (obp.direction == ObstacleBasePoint::up)
            ? ColorRGBA::orange
            : ((obp.direction == ObstacleBasePoint::down) ? ColorRGBA::yellow : ((obp.direction == ObstacleBasePoint::left) ? ColorRGBA::magenta : ColorRGBA(80, 200, 200)));
        CIRCLE("representation:ObstacleBasePoints:Image:Lower", obp.pointInImage.x(), obp.pointInImage.y(), 10, 3, Drawings::solidPen, color, Drawings::noBrush, ColorRGBA::gray);
      }
    }
  }

  DEBUG_DRAWING("representation:ObstacleBasePoints:Image:Upper", "drawingOnImage")
  {
    for (auto const& obp : basePoints)
    {
      if (obp.upperCam)
      {
        ColorRGBA color = (obp.direction == ObstacleBasePoint::up)
            ? ColorRGBA::orange
            : ((obp.direction == ObstacleBasePoint::down) ? ColorRGBA::yellow : ((obp.direction == ObstacleBasePoint::left) ? ColorRGBA::magenta : ColorRGBA(80, 200, 200)));
        CIRCLE("representation:ObstacleBasePoints:Image:Upper", obp.pointInImage.x(), obp.pointInImage.y(), 10, 3, Drawings::solidPen, color, Drawings::noBrush, ColorRGBA::gray);
      }
    }
  }
}

void RobotsPercept::draw() const
{
  //Image
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Image:Lower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Image:Upper", "drawingOnImage");

  for (auto const& robotEstimate : robots)
  {
    ColorRGBA color = robotEstimate.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green : (robotEstimate.robotType == RobotEstimate::opponentRobot ? ColorRGBA::red : ColorRGBA::black);
    ColorRGBA recallColor = robotEstimate.robotType == RobotEstimate::unknownRobot ? ColorRGBA::white : ColorRGBA::black;
    const char* srcStr = "";
    if (robotEstimate.source == RobotEstimate::closeRobotDetection)
      srcStr = "C";
    if (robotEstimate.source == RobotEstimate::segmentor)
      srcStr = "S";
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << robotEstimate.validity * 100.f << "%" << (robotEstimate.keeper ? " Keeper" : "") << (robotEstimate.fromUpperImage ? " U" : " L") << srcStr;

    if (robotEstimate.fromUpperImage)
    {
      RECTANGLE("representation:RobotsPercept:Image:Upper",
          robotEstimate.imageUpperLeft.x(),
          robotEstimate.imageUpperLeft.y(),
          robotEstimate.imageLowerRight.x(),
          robotEstimate.imageLowerRight.y(),
          5,
          Drawings::solidPen,
          color);
      if (robotEstimate.source == RobotEstimate::recallAcceptance)
      {
        RECTANGLE("representation:RobotsPercept:Image:Upper",
            robotEstimate.imageUpperLeft.x(),
            robotEstimate.imageUpperLeft.y(),
            robotEstimate.imageLowerRight.x(),
            robotEstimate.imageLowerRight.y(),
            1,
            Drawings::dashedPen,
            recallColor);
      }
      if (robotEstimate.imageUpperLeft.y() > (480 - robotEstimate.imageLowerRight.y()))
      {
        DRAWTEXT("representation:RobotsPercept:Image:Upper", robotEstimate.imageUpperLeft.x(), robotEstimate.imageUpperLeft.y() - 25, 13, color, ss.str());
      }
      else
      {
        DRAWTEXT("representation:RobotsPercept:Image:Upper", robotEstimate.imageUpperLeft.x(), robotEstimate.imageLowerRight.y() + 25, 13, color, ss.str());
      }
    }
    else
    {
      RECTANGLE("representation:RobotsPercept:Image:Lower",
          robotEstimate.imageUpperLeft.x(),
          robotEstimate.imageUpperLeft.y(),
          robotEstimate.imageLowerRight.x(),
          robotEstimate.imageLowerRight.y(),
          5,
          Drawings::solidPen,
          color);
      if (robotEstimate.source == RobotEstimate::recallAcceptance)
      {
        RECTANGLE("representation:RobotsPercept:Image:Lower",
            robotEstimate.imageUpperLeft.x(),
            robotEstimate.imageUpperLeft.y(),
            robotEstimate.imageLowerRight.x(),
            robotEstimate.imageLowerRight.y(),
            1,
            Drawings::dashedPen,
            recallColor);
      }
      if (robotEstimate.imageUpperLeft.y() < 0)
      {
        //const float xFraction = 2;
        //const float yFraction = 2;
        //Vector2i ulTransformed(static_cast<int>(round(robotEstimate.imageUpperLeft.x() * xFraction)), static_cast<int>(round(480 + robotEstimate.imageUpperLeft.y() * yFraction)));
        //Vector2i lrTransformed(static_cast<int>(round(robotEstimate.imageLowerRight.x() * xFraction)), static_cast<int>(round(480 + robotEstimate.imageLowerRight.y() * yFraction)));
        Vector2i ulTransformed, lrTransformed;
        robotEstimate.getUpperImageCoordinates(ulTransformed, lrTransformed);

        RECTANGLE("representation:RobotsPercept:Image:Upper", ulTransformed.x(), ulTransformed.y(), lrTransformed.x(), lrTransformed.y(), 5, Drawings::solidPen, color);
        DRAWTEXT("representation:RobotsPercept:Image:Upper", ulTransformed.x(), lrTransformed.y() + 25, 13, color, ss.str());
      }

      if (robotEstimate.imageUpperLeft.y() > (240 - robotEstimate.imageLowerRight.y()))
      {
        DRAWTEXT("representation:RobotsPercept:Image:Lower", robotEstimate.imageUpperLeft.x(), robotEstimate.imageUpperLeft.y() - 25, 13, color, ss.str());
      }
      else
      {
        DRAWTEXT("representation:RobotsPercept:Image:Lower", robotEstimate.imageUpperLeft.x(), robotEstimate.imageLowerRight.y() + 25, 13, color, ss.str());
      }
    }
  }

  //Field
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:RobotsPercept:ConvexHull", "drawingOnField");
  for (auto const& robotEstimate : robots)
  {
    ColorRGBA color = robotEstimate.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green : (robotEstimate.robotType == RobotEstimate::opponentRobot ? ColorRGBA::red : ColorRGBA::black);
    CIRCLE("representation:RobotsPercept:Field", robotEstimate.locationOnField.translation.x(), robotEstimate.locationOnField.translation.y(), 40, 2, Drawings::solidPen, color, Drawings::solidBrush, color);

    if (robotEstimate.convexHull.size() <= 1)
      continue;
    for (int j = 0; j < static_cast<int>(robotEstimate.convexHull.size()) - 1; j++)
    {
      Vector2f p1 = robotEstimate.convexHull[j];
      Vector2f p2 = robotEstimate.convexHull[j + 1];

      LINE("representation:RobotsPercept:ConvexHull", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
    }
    Vector2f p1 = robotEstimate.convexHull[0];
    Vector2f p2 = robotEstimate.convexHull[robotEstimate.convexHull.size() - 1];
    LINE("representation:RobotsPercept:ConvexHull", p1.x(), p1.y(), p2.x(), p2.y(), 3, Drawings::solidPen, ColorRGBA::blue);
  }
};

void RobotEstimate::getUpperImageCoordinates(Vector2i& ulRes, Vector2i& lrRes) const
{
  if (fromUpperImage)
  {
    ulRes = imageUpperLeft, lrRes = imageLowerRight;
  }
  else
  {
    ImageCoordinateTransformations::toUpper(imageUpperLeft, ulRes);
    ImageCoordinateTransformations::toUpper(imageLowerRight, lrRes);
  }
}
