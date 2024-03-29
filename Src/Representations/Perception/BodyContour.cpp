/**
 * @file BodyContour.h
 * The file implements a struct that represents the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "BodyContour.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/CameraInfo.h"

BodyContour::Line::Line(const Vector2i& p1, const Vector2i& p2) : p1(p1.x() < p2.x() ? p1 : p2), p2(p1.x() < p2.x() ? p2 : p1) {}

void BodyContour::clipBottom(int x, int& y) const
{
  int yIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if (i->yAt(x, yIntersection) && yIntersection < y)
      y = yIntersection;
}

void BodyContour::clipBottom(int x, int& y, int imageHeight) const
{
  clipBottom(x, y);

  //clippedY can be outside the image
  if (y < 0)
    y = 0;
  else if (y >= imageHeight)
    y = imageHeight - 1;
}

void BodyContour::clipLeft(int& x, int y) const
{
  int xIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if (i->p1.y() > i->p2.y())
    {
      if (i->xAt(y, xIntersection) && xIntersection > x)
        x = xIntersection;
      else if (i->p2.y() <= y && i->p2.x() > x) // below a segment, clip anyway
        x = i->p2.x();
    }
}

void BodyContour::clipRight(int& x, int y) const
{
  int xIntersection;
  for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if (i->p1.y() < i->p2.y())
    {
      if (i->xAt(y, xIntersection) && xIntersection < x)
        x = xIntersection;
      else if (i->p1.y() <= y && i->p1.x() < x) // below a segment, clip anyway
        x = i->p1.x();
    }
}

void BodyContour::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BodyContour", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BodyContour:maxY", "drawingOnImage");
  COMPLEX_DRAWING("representation:BodyContour")
  {
    for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
      LINE("representation:BodyContour", i->p1.x(), i->p1.y(), i->p2.x(), i->p2.y(), 1, Drawings::solidPen, ColorRGBA(255, 0, 255));

    const CameraInfo& cameraInfo = Blackboard::get<CameraInfo>();
    for (int x = 0; x < cameraInfo.width; x += 10)
    {
      int y = cameraInfo.height;
      clipBottom(x, y);
      LINE("representation:BodyContour", x, y, x, cameraInfo.height, 1, Drawings::solidPen, ColorRGBA(255, 0, 255));
    }
  }
  COMPLEX_DRAWING("representation:BodyContour:maxY")
  {
    const CameraInfo& cameraInfo = Blackboard::get<CameraInfo>();
    int maxY = getMaxY(cameraInfo);
    LINE("representation:BodyContour:maxY", 0, maxY, cameraInfo.width - 1, maxY, 1, Drawings::solidPen, ColorRGBA(255, 0, 255));
  }
}

void BodyContourUpper::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BodyContourUpper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BodyContourUpper:maxY", "drawingOnImage");
  COMPLEX_DRAWING("representation:BodyContourUpper")
  {
    for (std::vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
      LINE("representation:BodyContourUpper", i->p1.x(), i->p1.y(), i->p2.x(), i->p2.y(), 1, Drawings::solidPen, ColorRGBA(255, 0, 255));

    const CameraInfo& cameraInfo = Blackboard::get<CameraInfoUpper>();
    for (int x = 0; x < cameraInfo.width; x += 10)
    {
      int y = cameraInfo.height;
      clipBottom(x, y);
      LINE("representation:BodyContourUpper", x, y, x, cameraInfo.height, 1, Drawings::solidPen, ColorRGBA(255, 0, 255));
    }
  }
  COMPLEX_DRAWING("representation:BodyContourUpper:maxY")
  {
    const CameraInfo& cameraInfo = Blackboard::get<CameraInfoUpper>();
    int maxY = getMaxY(cameraInfo);
    LINE("representation:BodyContourUpper:maxY", 0, maxY, cameraInfo.width - 1, maxY, 1, Drawings::solidPen, ColorRGBA(255, 0, 255));
  }
}


int BodyContour::getMaxY(const CameraInfo& cameraInfo) const
{
  int y = cameraInfo.height - 1;
  clipBottom(0, y);
  clipBottom(cameraInfo.width - 1, y);

  if (y < 0)
  {
    //no need to continue
    return 0;
  }

  for (const Line& line : lines)
  {
    if (line.p1.y() >= 0 && line.p1.x() >= 0 && line.p1.x() < cameraInfo.width && line.p1.y() < y)
      y = line.p1.y();
    if (line.p2.y() >= 0 && line.p2.x() >= 0 && line.p2.x() < cameraInfo.width && line.p2.y() < y)
      y = line.p2.y();
  }
  return y;
}

bool BodyContour::isValidPoint(const Vector2i& point) const
{
  Vector2i copy(point);
  clipBottom(copy.x(), copy.y());
  if (point.y() != copy.y())
    return false;

  clipLeft(copy.x(), copy.y());
  if (point.x() > copy.x())
    return false;
  if (point.x() < copy.x())
    return true;

  clipRight(copy.x(), copy.y());
  if (point.x() < copy.x())
    return false;

  return true;
}
