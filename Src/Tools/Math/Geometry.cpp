/**
 * @file Math/Geometry.cpp
 * Implemets class Geometry
 *
 * @author <A href=mailto:juengel@informatik.hu-berlin.de>Matthias Jüngel</A>
 * @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
 */

#include "Geometry.h"
#include "Transformation.h"
#include "Approx.h"
#include "RotationMatrix.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <cstdlib>

using namespace std;

float Geometry::angleTo(const Pose2f& from, const Vector2f& to)
{
  const Pose2f relPos = Pose2f(to) - from;
  return atan2(relPos.translation.y(), relPos.translation.x());
}

float Geometry::angleBetween(const Vector2f& direction1, const Vector2f& direction2)
{
  Pose2f pose(0.f, 0.f, 0.f);
  pose.rotate(angleTo(pose, direction1));
  return angleTo(pose, direction2);
}

void Geometry::Line::normalizeDirection()
{
  direction.normalize();
}

Geometry::Circle Geometry::getCircle(const Vector2i& point1, const Vector2i& point2, const Vector2i& point3)
{
  const float x1 = static_cast<float>(point1.x());
  const float y1 = static_cast<float>(point1.y());
  const float x2 = static_cast<float>(point2.x());
  const float y2 = static_cast<float>(point2.y());
  const float x3 = static_cast<float>(point3.x());
  const float y3 = static_cast<float>(point3.y());

  Circle circle;
  const float temp = x2 * y1 - x3 * y1 - x1 * y2 + x3 * y2 + x1 * y3 - x2 * y3;

  if (temp == 0)
    circle.radius = 0;
  else
    circle.radius = 0.5f * sqrt(((sqr(x1 - x2) + sqr(y1 - y2)) * (sqr(x1 - x3) + sqr(y1 - y3)) * (sqr(x2 - x3) + sqr(y2 - y3))) / sqr(temp));
  if (temp == 0)
    circle.center.x() = 0;
  else
    circle.center.x() = (sqr(x3) * (y1 - y2) + (sqr(x1) + (y1 - y2) * (y1 - y3)) * (y2 - y3) + sqr(x2) * (-y1 + y3)) / (-2.0f * temp);
  if (temp == 0)
    circle.center.y() = 0;
  else
    circle.center.y() = (sqr(x1) * (x2 - x3) + sqr(x2) * x3 + x3 * (-sqr(y1) + sqr(y2)) - x2 * (+sqr(x3) - sqr(y1) + sqr(y3)) + x1 * (-sqr(x2) + sqr(x3) - sqr(y2) + sqr(y3))) / (2.0f * temp);
  return circle;
}

void Geometry::PixeledLine::calculatePixels(int x1, int y1, int x2, int y2)
{
  ASSERT(empty()); // only call from constructors
  if (x1 == x2 && y1 == y2)
    emplace_back(x1, y1);
  else
  {
    if (std::abs(x2 - x1) > std::abs(y2 - y1))
    {
      const int sign = sgn(x2 - x1);
      const int numberOfPixels = std::abs(x2 - x1) + 1;
      reserve(numberOfPixels);
      for (int x = 0; x < numberOfPixels; ++x)
      {
        const int y = x * (y2 - y1) / (x2 - x1);
        emplace_back(x1 + x * sign, y1 + y * sign);
      }
    }
    else
    {
      const int sign = sgn(y2 - y1);
      const int numberOfPixels = std::abs(y2 - y1) + 1;
      reserve(numberOfPixels);
      for (int y = 0; y < numberOfPixels; ++y)
      {
        const int x = y * (x2 - x1) / (y2 - y1);
        emplace_back(x1 + x * sign, y1 + y * sign);
      }
    }
  }
}

bool Geometry::getIntersectionOfLines(const Line& line1, const Line& line2, Vector2f& intersection)
{
  if (line1.direction.y() * line2.direction.x() == line1.direction.x() * line2.direction.y())
    return false;

  intersection.x() = line1.base.x()
      + line1.direction.x() * (line1.base.y() * line2.direction.x() - line2.base.y() * line2.direction.x() + (-line1.base.x() + line2.base.x()) * line2.direction.y())
          / ((-line1.direction.y()) * line2.direction.x() + line1.direction.x() * line2.direction.y());

  intersection.y() = line1.base.y()
      + line1.direction.y() * (-line1.base.y() * line2.direction.x() + line2.base.y() * line2.direction.x() + (line1.base.x() - line2.base.x()) * line2.direction.y())
          / (line1.direction.y() * line2.direction.x() - line1.direction.x() * line2.direction.y());

  return true;
}

int Geometry::getIntersectionOfCircles(const Circle& c0, const Circle& c1, Vector2f& p1, Vector2f& p2)
{
  // dx and dy are the vertical and horizontal distances between
  // the circle centers.
  const float dx = c1.center.x() - c0.center.x();
  const float dy = c1.center.y() - c0.center.y();

  // Determine the straight-line distance between the centers.
  const float d = sqrt((dy * dy) + (dx * dx));

  // Check for solvability.
  if (d > (c0.radius + c1.radius)) // no solution. circles do not intersect.
    return 0;
  if (d < abs(c0.radius - c1.radius)) //no solution. one circle is contained in the other
    return 0;

  // 'point 2' is the point where the line through the circle
  // intersection points crosses the line between the circle
  // centers.

  // Determine the distance from point 0 to point 2.
  const float a = ((c0.radius * c0.radius) - (c1.radius * c1.radius) + (d * d)) / (2.0f * d);

  // Determine the coordinates of point 2.
  const float x2 = c0.center.x() + (dx * a / d);
  const float y2 = c0.center.y() + (dy * a / d);

  // Determine the distance from point 2 to either of the
  // intersection points.
  const float h = sqrt((c0.radius * c0.radius) - (a * a));

  // Now determine the offsets of the intersection points from
  // point 2.
  const float rx = -dy * (h / d);
  const float ry = dx * (h / d);

  // Determine the absolute intersection points.
  p1.x() = x2 + rx;
  p2.x() = x2 - rx;
  p1.y() = y2 + ry;
  p2.y() = y2 - ry;

  return p1 == p2 ? 1 : 2;
}

bool Geometry::isPointInCircle(const Circle& circle, const Vector2f& point)
{
  const float distanceToCenter = distance(circle.center, point);
  return distanceToCenter < circle.radius;
}

int Geometry::getIntersectionOfLineAndCircle(const Line& line, const Circle& circle, Vector2f& firstIntersection, Vector2f& secondIntersection)
{
  /* solves the following system of equations:
   *
   * (x - x_m)^2 + (y - y_m)^2 = r^2
   * p + l * v = [x, y]
   *
   * where [x_m, y_m] is the center of the circle,
   * p is line.base and v is line.direction and
   * [x, y] is an intersection point.
   * Solution was found with the help of maple.
   */
  const float divisor = line.direction.squaredNorm();
  const float p = 2 * (line.base.dot(line.direction) - circle.center.dot(line.direction)) / divisor;
  const float q = ((line.base - circle.center).squaredNorm() - sqr(circle.radius)) / divisor;
  const float p_2 = p / 2.0f;
  const float radicand = sqr(p_2) - q;
  if (radicand < 0)
    return 0;
  else
  {
    const float radix = sqrt(radicand);
    firstIntersection = line.base + line.direction * (-p_2 + radix);
    secondIntersection = line.base + line.direction * (-p_2 - radix);
    return radicand == 0 ? 1 : 2;
  }
}

bool Geometry::getIntersectionOfRaysFactor(const Line& ray1, const Line& ray2, float& factor)
{
  const float divisor = ray2.direction.x() * ray1.direction.y() - ray1.direction.x() * ray2.direction.y();
  if (divisor == 0)
    return false;
  const float k = (ray2.direction.y() * ray1.base.x() - ray2.direction.y() * ray2.base.x() - ray2.direction.x() * ray1.base.y() + ray2.direction.x() * ray2.base.y()) / divisor;
  const float l = (ray1.direction.y() * ray1.base.x() - ray1.direction.y() * ray2.base.x() - ray1.direction.x() * ray1.base.y() + ray1.direction.x() * ray2.base.y()) / divisor;
  if ((k >= 0) && (l >= 0) && (k <= 1) && (l <= 1))
  {
    factor = k;
    return true;
  }
  return false;
}

bool Geometry::isPointLeftOfLine(const Vector2f& point, const Line& line)
{
  return isPointLeftOfLine(point, line.base, line.base + line.direction);
}

bool Geometry::isPointLeftOfLine(const Vector2f& point, const Line& line, const float hysteresis)
{
  return isPointLeftOfLine(point, line.base, line.base + line.direction, hysteresis);
}

bool Geometry::isPointLeftOfLine(const Vector2f& point, const Vector2f& linePoint1, const Vector2f& linePoint2)
{
  return isPointLeftOfLine(point, linePoint1, linePoint2, 0.f);
}

/**
 * @param hysteresis has to be the squared value of the distance to one side in mm
 */
bool Geometry::isPointLeftOfLine(const Vector2f& point, const Vector2f& linePoint1, const Vector2f& linePoint2, const float hysteresis)
{
  return ((linePoint2.x() - linePoint1.x()) * (point.y() - linePoint1.y()) - (linePoint2.y() - linePoint1.y()) * (point.x() - linePoint1.x())) + hysteresis > 0;
}

float Geometry::getDistanceToLine(const Line& line, const Vector2f& point)
{
  Vector2f normal = Vector2f::Zero();
  normal.x() = line.direction.y();
  normal.y() = -line.direction.x();
  normal.normalize();

  const float c = normal.dot(line.base);

  return normal.dot(point) - c;
}

bool Geometry::getPerpendicularFootPointToLine(const Line& line, const Vector2f& point, Vector2f& perpendicularFootPoint)
{
  Vector2f normal = Vector2f::Zero();
  normal.x() = line.direction.y();
  normal.y() = -line.direction.x();
  normal.normalize();
  Line secondLine;
  secondLine.base = point;
  secondLine.direction = normal;

  return getIntersectionOfLines(line, secondLine, perpendicularFootPoint);
}

float Geometry::getDistanceToEdge(const Line& line, const Vector2f& point)
{
  if (line.direction.x() == 0 && line.direction.y() == 0)
    return distance(point, line.base);

  const float c = line.direction.dot(line.base);
  const float d = (line.direction.dot(point) - c) / (line.direction.dot(line.direction));

  if (d < 0)
    return distance(point, line.base);
  else if (d > 1.0f)
    return distance(point, line.base + line.direction);
  else
    return abs(getDistanceToLine(line, point));
}

float Geometry::distance(const Vector2f& point1, const Vector2f& point2)
{
  return (point2 - point1).norm();
}

float Geometry::distance(const Vector2i& point1, const Vector2i& point2)
{
  return (point2 - point1).cast<float>().norm();
}

void Geometry::calculateAnglesForPoint(const Vector2f& point, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& angles)
{
  const float factor = cameraInfo.focalLength;
  const Vector3f vectorToPoint(factor, cameraInfo.opticalCenter.x() - point.x(), cameraInfo.opticalCenter.y() - point.y());
  const Vector3f vectorToPointWorld = cameraMatrix.rotation * vectorToPoint;
  angles.x() = atan2(vectorToPointWorld.y(), vectorToPointWorld.x());
  angles.y() = atan2(vectorToPointWorld.z(), sqrt(sqr(vectorToPointWorld.x()) + sqr(vectorToPointWorld.y())));
}

bool Geometry::calculatePointByAngles(const Vector2f& angles, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& point)
{
  const Vector3f vectorToPointWorld(cos(angles.x()), sin(angles.x()), tan(angles.y()));
  const Vector3f vectorToPoint(cameraMatrix.rotation.inverse() * vectorToPointWorld);
  if (vectorToPoint.x() <= 0)
    return false;
  const float scale = cameraInfo.focalLength / vectorToPoint.x();
  point.x() = cameraInfo.opticalCenter.x() - vectorToPoint.y() * scale;
  point.y() = cameraInfo.opticalCenter.y() - vectorToPoint.z() * scale;
  return true;
}

bool Geometry::isPointInsideRectangle(const Vector2f& bottomLeftCorner, const Vector2f& topRightCorner, const Vector2f& point)
{
  return (bottomLeftCorner.x() <= point.x() && point.x() <= topRightCorner.x() && bottomLeftCorner.y() <= point.y() && point.y() <= topRightCorner.y());
}

bool Geometry::isPointInsideRectangle2(const Vector2f& corner1, const Vector2f& corner2, const Vector2f& point)
{
  const Vector2f bottomLeft(std::min(corner1.x(), corner2.x()), std::min(corner1.y(), corner2.y()));
  const Vector2f topRight(std::max(corner1.x(), corner2.x()), std::max(corner1.y(), corner2.y()));
  return isPointInsideRectangle(bottomLeft, topRight, point);
}

bool Geometry::isPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, const Vector2i& point)
{
  return (bottomLeftCorner.x() <= point.x() && point.x() <= topRightCorner.x() && bottomLeftCorner.y() <= point.y() && point.y() <= topRightCorner.y());
}

int Geometry::ccw(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2)
{
  const float dx1 = p1.x() - p0.x();
  const float dy1 = p1.y() - p0.y();
  const float dx2 = p2.x() - p0.x();
  const float dy2 = p2.y() - p0.y();
  if (dx1 * dy2 > dy1 * dx2)
    return 1;
  if (dx1 * dy2 < dy1 * dx2)
    return -1;
  // Now (dx1*dy2 == dy1*dx2) must be true:
  if ((dx1 * dx2 < 0.0f) || (dy1 * dy2 < 0.0f))
    return -1;
  if ((dx1 * dx1 + dy1 * dy1) >= (dx2 * dx2 + dy2 * dy2))
    return 0;
  return 1;
}

bool Geometry::isPointInsideConvexPolygon(const Vector2f polygon[], const int numberOfPoints, const Vector2f& point)
{
  const int orientation = ccw(polygon[0], polygon[1], point);
  if (orientation == 0)
    return true;
  for (int i = 1; i < numberOfPoints; i++)
  {
    const int currentOrientation = ccw(polygon[i], polygon[(i + 1) % numberOfPoints], point);
    if (currentOrientation == 0)
      return true;
    if (currentOrientation != orientation)
      return false;
  }
  return true;
}

bool Geometry::checkIntersectionOfLines(const Vector2f& l1p1, const Vector2f& l1p2, const Vector2f& l2p1, const Vector2f& l2p2)
{
  return (((ccw(l1p1, l1p2, l2p1) * ccw(l1p1, l1p2, l2p2)) <= 0) && ((ccw(l2p1, l2p2, l1p1) * ccw(l2p1, l2p2, l1p2)) <= 0));
}

bool Geometry::clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2i& point)
{
  bool clipped = false;
  if (point.x() < bottomLeftCorner.x())
  {
    point.x() = bottomLeftCorner.x();
    clipped = true;
  }
  if (point.x() > topRightCorner.x())
  {
    point.x() = topRightCorner.x();
    clipped = true;
  }
  if (point.y() < bottomLeftCorner.y())
  {
    point.y() = bottomLeftCorner.y();
    clipped = true;
  }
  if (point.y() > topRightCorner.y())
  {
    point.y() = topRightCorner.y();
    clipped = true;
  }
  return clipped;
}

bool Geometry::clipPointInsideRectangle(const Vector2i& bottomLeftCorner, const Vector2i& topRightCorner, Vector2f& point)
{
  bool clipped = false;
  if (point.x() < bottomLeftCorner.x())
  {
    point.x() = static_cast<float>(bottomLeftCorner.x());
    clipped = true;
  }
  if (point.x() > topRightCorner.x())
  {
    point.x() = static_cast<float>(topRightCorner.x());
    clipped = true;
  }
  if (point.y() < bottomLeftCorner.y())
  {
    point.y() = static_cast<float>(bottomLeftCorner.y());
    clipped = true;
  }
  if (point.y() > topRightCorner.y())
  {
    point.y() = static_cast<float>(topRightCorner.y());
    clipped = true;
  }
  return clipped;
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(const Vector2i& bottomLeft, const Vector2i& topRight, const Geometry::Line& line, Vector2i& point1, Vector2i& point2)
{
  int foundPoints = 0;
  Vector2f point[2];
  if (line.direction.x() != 0)
  {
    const float y1 = line.base.y() + (bottomLeft.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if ((y1 >= bottomLeft.y()) && (y1 <= topRight.y()))
    {
      point[foundPoints].x() = static_cast<float>(bottomLeft.x());
      point[foundPoints++].y() = y1;
    }
    const float y2 = line.base.y() + (topRight.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if ((y2 >= bottomLeft.y()) && (y2 <= topRight.y()))
    {
      point[foundPoints].x() = static_cast<float>(topRight.x());
      point[foundPoints++].y() = y2;
    }
  }
  if (line.direction.y() != 0)
  {
    const float x1 = line.base.x() + (bottomLeft.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if ((x1 >= bottomLeft.x()) && (x1 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x1;
      point[foundPoints].y() = static_cast<float>(bottomLeft.y());
      if ((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
    const float x2 = line.base.x() + (topRight.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if ((x2 >= bottomLeft.x()) && (x2 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x2;
      point[foundPoints].y() = static_cast<float>(topRight.y());
      if ((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
  }
  switch (foundPoints)
  {
  case 1:
    point1 = point[0].cast<int>();
    point2 = point1;
    foundPoints++;
    return true;
  case 2:
    if ((point[1] - point[0]).dot(line.direction) > 0)
    {
      point1 = point[0].cast<int>();
      point2 = point[1].cast<int>();
    }
    else
    {
      point1 = point[1].cast<int>();
      point2 = point[0].cast<int>();
    }
    return true;
  default:
    return false;
  }
}

bool Geometry::getIntersectionPointsOfLineAndRectangle(const Vector2f& bottomLeft, const Vector2f& topRight, const Geometry::Line& line, Vector2f& point1, Vector2f& point2)
{
  int foundPoints = 0;
  Vector2f point[2];
  if (line.direction.x() != 0)
  {
    const float y1 = line.base.y() + (bottomLeft.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if ((y1 >= bottomLeft.y()) && (y1 <= topRight.y()))
    {
      point[foundPoints].x() = bottomLeft.x();
      point[foundPoints++].y() = y1;
    }
    const float y2 = line.base.y() + (topRight.x() - line.base.x()) * line.direction.y() / line.direction.x();
    if ((y2 >= bottomLeft.y()) && (y2 <= topRight.y()))
    {
      point[foundPoints].x() = topRight.x();
      point[foundPoints++].y() = y2;
    }
  }
  if (line.direction.y() != 0)
  {
    const float x1 = line.base.x() + (bottomLeft.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if ((x1 >= bottomLeft.x()) && (x1 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x1;
      point[foundPoints].y() = bottomLeft.y();
      if ((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
      {
        foundPoints++;
      }
    }
    const float x2 = line.base.x() + (topRight.y() - line.base.y()) * line.direction.x() / line.direction.y();
    if ((x2 >= bottomLeft.x()) && (x2 <= topRight.x()) && (foundPoints < 2))
    {
      point[foundPoints].x() = x2;
      point[foundPoints].y() = topRight.y();
      if ((foundPoints == 0) || ((point[0] - point[1]).norm() > 0.1))
        foundPoints++;
    }
  }
  switch (foundPoints)
  {
  case 1:
    point1 = point[0];
    point2 = point1;
    foundPoints++;
    return true;
  case 2:
    if ((point[1] - point[0]).dot(line.direction) > 0)
    {
      point1 = point[0];
      point2 = point[1];
    }
    else
    {
      point1 = point[1];
      point2 = point[0];
    }
    return true;
  default:
    return false;
  }
}

#define CLIPLEFT 1 // 0001
#define CLIPRIGHT 2 // 0010
#define CLIPLOWER 4 // 0100
#define CLIPUPPER 8 // 1000

bool Geometry::clipLineWithRectangleCohenSutherland(const Vector2i& topLeft, const Vector2i& bottomRight, Vector2i& point1, Vector2i& point2)
{
  int K1 = 0, K2 = 0;

  const int dx = point2.x() - point1.x();
  const int dy = point2.y() - point1.y();

  if (point1.y() < topLeft.y())
    K1 = CLIPLOWER;
  if (point1.y() > bottomRight.y())
    K1 = CLIPUPPER;
  if (point1.x() < topLeft.x())
    K1 |= CLIPLEFT;
  if (point1.x() > bottomRight.x())
    K1 |= CLIPRIGHT;

  if (point2.y() < topLeft.y())
    K2 = CLIPLOWER;
  if (point2.y() > bottomRight.y())
    K2 = CLIPUPPER;
  if (point2.x() < topLeft.x())
    K2 |= CLIPLEFT;
  if (point2.x() > bottomRight.x())
    K2 |= CLIPRIGHT;

  while (K1 || K2)
  {
    if (K1 & K2)
      return false;

    if (K1)
    {
      if (K1 & CLIPLEFT)
      {
        point1.y() += (topLeft.x() - point1.x()) * dy / dx;
        point1.x() = topLeft.x();
      }
      else if (K1 & CLIPRIGHT)
      {
        point1.y() += (bottomRight.x() - point1.x()) * dy / dx;
        point1.x() = bottomRight.x();
      }
      if (K1 & CLIPLOWER)
      {
        point1.x() += (topLeft.y() - point1.y()) * dx / dy;
        point1.y() = topLeft.y();
      }
      else if (K1 & CLIPUPPER)
      {
        point1.x() += (bottomRight.y() - point1.y()) * dx / dy;
        point1.y() = bottomRight.y();
      }
      K1 = 0;

      if (point1.y() < topLeft.y())
        K1 = CLIPLOWER;
      if (point1.y() > bottomRight.y())
        K1 = CLIPUPPER;
      if (point1.x() < topLeft.x())
        K1 |= CLIPLEFT;
      if (point1.x() > bottomRight.x())
        K1 |= CLIPRIGHT;
    }

    if (K1 & K2)
      return false;

    if (K2)
    {
      if (K2 & CLIPLEFT)
      {
        point2.y() += (topLeft.x() - point2.x()) * dy / dx;
        point2.x() = topLeft.x();
      }
      else if (K2 & CLIPRIGHT)
      {
        point2.y() += (bottomRight.x() - point2.x()) * dy / dx;
        point2.x() = bottomRight.x();
      }
      if (K2 & CLIPLOWER)
      {
        point2.x() += (topLeft.y() - point2.y()) * dx / dy;
        point2.y() = topLeft.y();
      }
      else if (K2 & CLIPUPPER)
      {
        point2.x() += (bottomRight.y() - point2.y()) * dx / dy;
        point2.y() = bottomRight.y();
      }
      K2 = 0;

      if (point2.y() < topLeft.y())
        K2 = CLIPLOWER;
      if (point2.y() > bottomRight.y())
        K2 = CLIPUPPER;
      if (point2.x() < topLeft.x())
        K2 |= CLIPLEFT;
      if (point2.x() > bottomRight.x())
        K2 |= CLIPRIGHT;
    }
  }
  return true;
}

int Geometry::intersection(int a1, int b1, int a2, int b2, int value)
{
  if (a2 - a1 != 0)
    return static_cast<int>(b1 + static_cast<float>((value - a1)) / (a2 - a1) * (b2 - b1));
  else
    return 32767;
}

bool Geometry::calculateBallInImage(const Vector2f& ballOffset, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float ballRadius, Circle& circle)
{
  const Vector2f offset = ballOffset - cameraMatrix.translation.head<2>();
  const float distance = offset.norm();
  const float height = cameraMatrix.translation.z() - ballRadius;
  const float cameraDistance = sqrt(sqr(distance) + sqr(height));
  circle.center = Vector2f(atan2(offset.y(), offset.x()), -atan2(height, distance));
  if (cameraDistance >= ballRadius)
  {
    const float alpha = pi_2 - circle.center.y() - acos(ballRadius / cameraDistance);
    const float yBottom = -atan2(height + cos(alpha) * ballRadius, distance - sin(alpha) * ballRadius);
    const float beta = pi_2 - circle.center.y() + acos(ballRadius / cameraDistance);
    const float yTop = -atan2(height + cos(beta) * ballRadius, distance - sin(beta) * ballRadius);
    Vector2f top, bottom;
    if (!calculatePointByAngles(Vector2f(circle.center.x(), yTop), cameraMatrix, cameraInfo, top))
      return false;
    if (!calculatePointByAngles(Vector2f(circle.center.x(), yBottom), cameraMatrix, cameraInfo, bottom))
      return false;
    circle.center = (top + bottom) / 2.0f;
    circle.radius = (top - bottom).norm() / 2.0f;
    return true;
  }
  else
    return false;
}

void Geometry::computeFieldOfViewInFieldCoordinates(
    const RobotPose& robotPose, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions, std::vector<Vector2f>& p)
{
  if (p.size() < 4)
    p.resize(4);
  const Vector3f vectorToCenter(1, 0, 0);

  RotationMatrix r(cameraMatrix.rotation);
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  Vector3f vectorToCenterWorld = r * vectorToCenter;

  const float a1 = cameraMatrix.translation.x();
  const float a2 = cameraMatrix.translation.y();
  const float a3 = cameraMatrix.translation.z();
  float b1 = vectorToCenterWorld.x(), b2 = vectorToCenterWorld.y(), b3 = vectorToCenterWorld.z(), f = a3 / b3;
  Vector2f pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    p[0] = robotPose.translation;
  else
    p[0] = robotPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    p[1] = robotPose.translation;
  else
    p[1] = robotPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(
      4.f * fieldDimensions.xPosOpponentFieldBorder * fieldDimensions.xPosOpponentFieldBorder + 4.f * fieldDimensions.yPosLeftFieldBorder * fieldDimensions.yPosLeftFieldBorder);
  if (f > 0.f)
    p[2] = robotPose.translation + Vector2f(maxDist, 0).rotate(robotPose.rotation + (-cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    p[2] = robotPose * pof;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if (f > 0.f)
    p[3] = robotPose.translation + Vector2f(maxDist, 0).rotate(robotPose.rotation + (cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    p[3] = robotPose * pof;
}

float Geometry::angleSizeToPixelSize(float angleSize, const CameraInfo& cameraInfo)
{
  return cameraInfo.focalLength * tan(angleSize);
}

float Geometry::pixelSizeToAngleSize(float pixelSize, const CameraInfo& cameraInfo)
{
  return atan(pixelSize * cameraInfo.focalLengthInv);
}

float Geometry::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels)
{
  const float xFactor = cameraInfo.focalLength;
  return sizeInReality * xFactor / (sizeInPixels + 0.000001f);
}

float Geometry::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels, float centerX, float centerY)
{
  const float mx = centerX;
  const float my = centerY;
  const float cx = cameraInfo.opticalCenter.x();
  const float cy = cameraInfo.opticalCenter.y();
  const float focalLenPow2 = cameraInfo.focalLenPow2;
  const float sqrImgRadius = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
  const float imgDistance = sqrt(focalLenPow2 + sqrImgRadius);
  return imgDistance * sizeInReality / (sizeInPixels + 0.000001f);
}

float Geometry::getSizeByDistance(const CameraInfo& cameraInfo, float sizeInReality, float distance)
{
  const float xFactor = cameraInfo.focalLength;
  return sizeInReality / distance * xFactor;
}

Geometry::Line Geometry::calculateHorizon(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo)
{
  const RowVector3f bottomRow = cameraMatrix.rotation.bottomRows(1);
  const float r31 = bottomRow.x();
  const float r32 = bottomRow.y();
  const float r33 = bottomRow.z() == 0 ? 0.00001f : bottomRow.z();

  const float v1 = cameraInfo.focalLength;
  const float v2 = cameraInfo.opticalCenter.x();
  const float v3 = cameraInfo.opticalCenter.y();
  float x1 = 0;
  float x2 = static_cast<float>(cameraInfo.width - 1);
  float y1 = (v3 * r33 + r31 * v1 + r32 * v2) / r33;
  float y2 = (v3 * r33 + r31 * v1 - r32 * v2) / r33;

  // Mirror ends of horizon if Camera rotated to the left
  if ((cameraMatrix.rotation * Vector3f(0, 0, 1)).z() < 0)
  {
    float t = x1;
    x1 = x2;
    x2 = t;
    t = y1;
    y1 = y2;
    y2 = t;
  }

  Line horizon;
  horizon.base.x() = (x1 + x2) / 2.f;
  horizon.base.y() = (y1 + y2) / 2.f;
  horizon.direction.x() = x2 - x1;
  horizon.direction.y() = y2 - y1;
  horizon.normalizeDirection();
  return horizon;
}

void Geometry::linspaced(const Vector2f& start, const Vector2f& stop, unsigned count, std::vector<Vector2f>& out)
{
  out.resize(count);
  const Vector2f distance = stop - start;
  const Vector2f dt = distance / (static_cast<float>(count) - 1.f);
  for (unsigned i = 0; i < count; ++i)
    out[i] = start + dt * static_cast<float>(i);
}

bool Geometry::isPointInsideTriangle(const float x1, const float y1, const float x2, const float y2, const float x3, const float y3, const float px, const float py)
{
  // calc  barycentric coordinates
  const float alpha = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
  const float beta = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));
  const float gamma = 1.0f - alpha - beta;
  return alpha > 0 && beta > 0 && gamma > 0;
}

int Geometry::calculateLineSize(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float fieldLinesWidth)
{
  Vector2f pointOnField; //position on field, relative to robot
  if (Transformation::imageToRobot(pointInImage.x(), pointInImage.y(), cameraMatrix, cameraInfo, pointOnField))
  {
    float distance = std::sqrt(sqr(cameraMatrix.translation.z()) + sqr(pointOnField.norm()));
    return static_cast<int>(Geometry::getSizeByDistance(cameraInfo, fieldLinesWidth, distance));
  }
  else
  {
    return 0;
  }
}

float Geometry::calculateLineSizePrecise(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float fieldLinesWidth)
{
  Vector2f pointOnField; //position on field, relative to robot
  if (Transformation::imageToRobot(pointInImage.x(), pointInImage.y(), cameraMatrix, cameraInfo, pointOnField))
  {
    float distance = std::sqrt(sqr(cameraMatrix.translation.z()) + sqr(pointOnField.norm()));
    return Geometry::getSizeByDistance(cameraInfo, fieldLinesWidth, distance);
  }
  else
  {
    return 0;
  }
}

bool Geometry::computeCircleOnFieldLevenbergMarquardt(const std::vector<Vector2f>& circlePoints, Geometry::Circle& circle)
{
  int number = static_cast<int>(circlePoints.size());

  if (number < 3)
    return false;
  float Mx(0), My(0), Mxx(0), Myy(0), Mxy(0), Mz(0), Mxz(0), Myz(0);
  for (int i = 0; i < number; ++i)
  {
    float x = circlePoints[i].x();
    float y = circlePoints[i].y();
    float xx = x * x;
    float yy = y * y;
    float z = xx + yy;
    Mx += x;
    My += y;
    Mxx += xx;
    Myy += yy;
    Mxy += x * y;
    Mz += z;
    Mxz += x * z;
    Myz += y * z;
  }

  // Construct and solve matrix (might fail and throw exception).
  // Result will be center and radius of circle.
  try
  {
    Matrix3f M;
    M << Mxx, Mxy, Mx, Mxy, Myy, My, Mx, My, static_cast<float>(number);

    Vector3f v;
    v[0] = -Mxz;
    v[1] = -Myz;
    v[2] = -Mz;

    Vector3f BCD;
    BCD = M.ldlt().solve(v);
    circle.center.x() = (-BCD[0] / 2.f);
    circle.center.y() = (-BCD[1] / 2.f);
    float radicand = BCD[0] * BCD[0] / 4.f + BCD[1] * BCD[1] / 4.f - BCD[2];
    if (radicand < 0.f)
      return false;
    circle.radius = sqrt(radicand);
  }
  catch (...)
  {
    return false;
  }
  return true;
}

Geometry::Line Geometry::calculateLineByLinearRegression(const std::vector<Vector2f>& pointsForLine, float& avgError, float& biggestError)
{
  Geometry::Line result;

  float x_avg = 0;
  float y_avg = 0;

  int n = (int)pointsForLine.size();
  if (n == 1)
  {
    avgError = 0.f;
    biggestError = 0.f;
    result.base.x() = pointsForLine[0].x();
    result.base.y() = pointsForLine[0].y();
    result.direction.x() = 1;
    result.direction.y() = 1;
    return result;
  }

  for (int i = 0; i < n; i++)
  {
    x_avg += pointsForLine[i].x();
    y_avg += pointsForLine[i].y();
  }
  x_avg /= n;
  y_avg /= n;


  float angle = std::abs((pointsForLine[0] - pointsForLine[n - 1]).angle());
  if (angle < pi_4 || angle > pi3_4)
  {
    // solve for:
    // y = a + b*x
    float x_diff;
    float ss_xy = 0;
    float ss_xx = 0;

    for (int i = 0; i < n; i++)
    {
      x_diff = pointsForLine[i].x() - x_avg;
      ss_xy += x_diff * (pointsForLine[i].y() - y_avg);
      ss_xx += x_diff * x_diff;
    }

    result.direction.x() = 1;
    result.direction.y() = ss_xy / ss_xx;
    result.base.x() = 0;
    result.base.y() = y_avg - result.direction.y() * x_avg;

    result.base += result.direction * x_avg;
  }
  else
  {
    // solve for:
    // x = a + b*y
    float y_diff;
    float ss_xy = 0;
    float ss_yy = 0;

    for (int i = 0; i < n; i++)
    {
      y_diff = pointsForLine[i].y() - y_avg;
      ss_xy += y_diff * (pointsForLine[i].x() - x_avg);
      ss_yy += y_diff * y_diff;
    }

    result.direction.x() = ss_xy / ss_yy;
    result.direction.y() = 1;
    result.base.x() = x_avg - result.direction.x() * y_avg;
    result.base.y() = 0;

    result.base += result.direction * y_avg;
  }

  // use euklid distance for error calculation
  Vector2f normal = result.direction;
  normal.rotateLeft();
  normal.normalize();
  avgError = 0;
  biggestError = 0;
  for (int i = 0; i < n; i++)
  {
    float error = std::abs(getDistanceToLine(result, Vector2f(pointsForLine[i].x(), pointsForLine[i].y())));
    avgError += error;
    if (biggestError < error)
    {
      biggestError = error;
    }
  }
  avgError /= n;


  return result;
}

bool Geometry::wouldPointBeVisible(const Vector2f& relPosOnField, const Angle maxHeadPan, const CameraInfo& cameraInfo)
{
  // TODO: use body contour?
  Angle rotateAngle = relPosOnField.angle();
  const Angle rotateAngleAbs = std::abs(rotateAngle);
  const Angle safeOpeningAngle = cameraInfo.openingAngleWidth / 2 - 5_deg;
  if (
      // Point invisible with any possible head pan.
      rotateAngleAbs > (maxHeadPan + safeOpeningAngle)
      // Point hidden by own shoulder.
      || (rotateAngleAbs > 70_deg && rotateAngleAbs < 110_deg && relPosOnField.norm() < 3000) || (rotateAngleAbs > 75_deg && rotateAngleAbs < 115_deg && relPosOnField.norm() < 3500))
    return false;
  return true;
}

bool Geometry::ballShouldBeVisibleInImage(const Vector2f& relBallPositionOnField, float ballRadius, const CameraMatrix& theCameraMatrix, const CameraInfo& theCameraInfo)
{
  if (theCameraMatrix.isValid)
  {
    Vector3f ballRel;
    ballRel << relBallPositionOnField, ballRadius; // use ballRadius as height (z-coordinate)
    Vector2f ballInImage;
    if (Transformation::robotToImage(ballRel, theCameraMatrix, theCameraInfo, ballInImage))
    {
      const float ballDistance = (theCameraMatrix.inverse() * ballRel).norm();
      const float ballRadiusInImage = Geometry::getSizeByDistance(theCameraInfo, ballRadius, ballDistance);
      const float offset = std::ceil(ballRadiusInImage);
      if (ballInImage.x() >= offset && ballInImage.x() < theCameraInfo.width - offset && ballInImage.y() >= offset && ballInImage.y() < theCameraInfo.height - offset)
        return true;
    }
  }
  return false;
}
