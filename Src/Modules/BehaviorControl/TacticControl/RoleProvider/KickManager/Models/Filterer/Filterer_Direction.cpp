#include "Filterer.h"

#include "Tools/Math/Geometry.h"

Filterer& Filterer::filterOutsideKickRange(const KickRange& kickRange) // todo cant copy
{
  outsideKickRangeDirectionFilter = [&kickRange](const SelectableDirection& selectableDirection)
  {
    return !kickRange.istValidDirection(selectableDirection.angle);
  };
  outsideKickRangeTargetFilter = [&kickRange](const SelectableTarget& selectableTarget)
  {
    return !kickRange.istValidTarget(selectableTarget.target);
  };
  return *this;
}

Filterer& Filterer::filterBetweenAngles(const Vector2f& ballPosition, const Angle leftAngle, const Angle rightAngle)
{
  ASSERT(leftAngle > -181_deg);
  ASSERT(leftAngle < 181_deg);
  ASSERT(rightAngle > -181_deg);
  ASSERT(rightAngle < 181_deg);
  ASSERT(MathUtils::getAngleSmallestDiff(leftAngle, rightAngle) < 181_deg);
  betweenAnglesFilter = [ballPosition, leftAngle, rightAngle](const SelectableDirection& selectableDirection)
  {
    return MathUtils::isBetweenAngles(selectableDirection.angle, leftAngle, rightAngle);
  };
  return *this;
}

Filterer& Filterer::filterToOwnGoal(const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
{
  const Vector2f leftGoalPostPosition = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal + 500.f};
  const Vector2f rightGoalPostPosition = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal - 500.f};

  const Angle leftGoalPostAngle = (leftGoalPostPosition - ballPosition).angle();
  const Angle rightGoalPostAngle = (rightGoalPostPosition - ballPosition).angle();

  ASSERT(leftGoalPostAngle > -181_deg);
  ASSERT(leftGoalPostAngle < 181_deg);
  ASSERT(rightGoalPostAngle > -181_deg);
  ASSERT(rightGoalPostAngle < 181_deg);
  ASSERT(MathUtils::getAngleSmallestDiff(leftGoalPostAngle, rightGoalPostAngle) < 181_deg);

  toOwnGoalFilter = [leftGoalPostAngle, rightGoalPostAngle](const SelectableDirection& selectableDirection)
  {
    if (std::abs(leftGoalPostAngle) < 90_deg)
    {
      ASSERT(std::abs(rightGoalPostAngle) < 90_deg);
      return false; // is behind goal line
    }
    return MathUtils::isBetweenAngles(selectableDirection.angle, rightGoalPostAngle, leftGoalPostAngle);
  };
  return *this;
}
