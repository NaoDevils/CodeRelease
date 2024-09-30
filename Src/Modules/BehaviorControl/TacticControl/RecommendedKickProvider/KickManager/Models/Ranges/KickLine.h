#pragma once

#include "KickRange.h"
#include "DistanceRequirement.h"
#include <optional>
#include <utility>

class KickLine : public KickRange
{

public:
  KickLine(const Vector2f& ballPosition, const Vector2f& point1, const Vector2f& point2, const DistanceRequirement distanceRequirement, const float additionalDistance)
      : KickRange(ballPosition), point1(point1), point2(point2), distanceRequirement(distanceRequirement), additionalDistance(additionalDistance)
  {
    this->point1 = point1;
    this->point2 = point2;

    draw();
  }

  [[nodiscard]] bool istValidDirection(const Angle& angle) const override
  {
    const Angle point1Angle = (point1 - ballPosition).angle();
    const Angle point2Angle = (point2 - ballPosition).angle();
    const auto [leftAngle, rightAngle] = MathUtils::getLeftAndRightAngle(point1Angle, point2Angle);
    return MathUtils::isBetweenAngles(angle, leftAngle, rightAngle);
  }

  /**
   * assumes the direction is valid
   */
  [[nodiscard]] bool istValidTarget(const Vector2f& target) const override
  {
    ASSERT(istValidDirection((target - ballPosition).angle()));
    const Geometry::Line line1 = {point1, point2 - point1};
    const Geometry::Line line2 = {ballPosition, target - ballPosition};
    Vector2f intersection;
    VERIFY(Geometry::getIntersectionOfLines(line1, line2, intersection)); // otherwise should get filtered out with the test for valid direction

    const float targetDistance = Geometry::distance(ballPosition, target);
    const float intersectionDistance = Geometry::distance(ballPosition, intersection);

    switch (distanceRequirement)
    {
    case mayShorterOrFurther:
      return true;
    case onPoint:
      return intersectionDistance - additionalDistance <= targetDistance && targetDistance <= intersectionDistance + additionalDistance;
    case mustFurther:
      return targetDistance >= intersectionDistance + additionalDistance;
    case mustShorter:
      return targetDistance <= intersectionDistance - additionalDistance;
    default:
      throw std::logic_error("");
    }
  }

  void draw() const override
  {
    KickRange::draw(ballPosition, point1);
    KickRange::draw(ballPosition, point2);
  }

  Vector2f point1;
  Vector2f point2;
  DistanceRequirement distanceRequirement;
  float additionalDistance; // depending on the distanceRequirement a distance that is allowed or required on one or both directions
};
