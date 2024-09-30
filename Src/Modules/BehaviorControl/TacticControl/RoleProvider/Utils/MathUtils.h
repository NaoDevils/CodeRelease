#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "Tools/ColorRGBA.h"

class MathUtils
{
public:
  static ColorRGBA valueToRGB(const float value)
  {
    ASSERT(0 <= value);
    ASSERT(value <= 1);
    return valuesToRGB(1.f, 0.f, 0.f, 155.f * value);
  }

  static ColorRGBA valuesToRGB(const float valueRed, const float valueGreen, const float valueBlue, const float alpha)
  {
    ASSERT(0 <= valueRed);
    ASSERT(valueRed <= 1);
    ASSERT(0 <= valueGreen);
    ASSERT(valueGreen <= 1);
    ASSERT(0 <= valueBlue);
    ASSERT(valueBlue <= 1);
    ASSERT(0 <= alpha);
    ASSERT(alpha <= 255);
    return {(unsigned char)(255 * valueRed), (unsigned char)(255 * valueGreen), (unsigned char)(255 * valueBlue), (unsigned char)(alpha)};
  }

  static int clamp_i(const int current, const int min, const int max) { return current < min ? min : current > max ? max : current; }

  static float clamp_f(const float current, const float min, const float max) { return current < min ? min : current > max ? max : current; }

  static void stretch(std::vector<float>& values)
  {
    const size_t size = values.size();

    float min = +std::numeric_limits<float>::max();
    float max = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < size; i++)
    {
      const float heat = values.at(i);
      if (heat < min)
      {
        min = heat;
      }
      if (heat > max)
      {
        max = heat;
      }
    }

    for (unsigned int i = 0; i < size; ++i)
    {
      float stretchedValue = (values.at(i) - min) / (max - min);
      stretchedValue = MathUtils::clamp_f(stretchedValue, 0.f, 1.f);
      values.at(i) = stretchedValue;
    }
  }

  inline static Vector2f angleToNormalizedVector(const Angle angle)
  {
    const Vector2f vector = angleToVector(angle);
    if (angle < 0)
    {
      return -vector;
    }
    else
    {
      return vector;
    }
  }

  /**
   * @return a vector that is not normalized
   */
  inline static Vector2f angleToVector(const Angle angle) { return {(float)std::cos(angle), (float)std::sin(angle)}; }

  /**
   * Assumes every angle is normalized and that the difference is a maximum of 180_deg
   */
  inline static bool isBetweenAngles(Angle angle, Angle leftAngle, Angle rightAngle)
  {
    ASSERT(angle > -181_deg);
    ASSERT(angle < 181_deg);
    ASSERT(leftAngle > -181_deg);
    ASSERT(leftAngle < 181_deg);
    ASSERT(rightAngle > -181_deg);
    ASSERT(rightAngle < 181_deg);
    if (leftAngle > rightAngle)
    {
      return leftAngle > angle && angle > rightAngle;
    }
    return angle < leftAngle || angle > rightAngle;
  }

  inline static Angle getLeftAngle(Angle angle1, Angle angle2)
  {
    angle1 = angle1.normalize();
    angle2 = angle2.normalize();

    const Angle angleDiff = std::fabs(Angle(angle1 - angle2));

    if (angle1 > 0 && angle2 < 0)
    {
      return angleDiff < 180_deg ? angle1 : angle2;
    }

    if (angle2 > 0 && angle1 < 0)
    {
      return angleDiff < 180_deg ? angle2 : angle1;
    }

    return angle1 > angle2 ? angle1 : angle2;
  }

  inline static std::tuple<Angle, Angle> getLeftAndRightAngle(Angle angle1, Angle angle2)
  {
    angle1 = angle1.normalize();
    angle2 = angle2.normalize();

    const Angle angleDiff = std::fabs(Angle(angle1 - angle2));

    if (angle1 > 0 && angle2 < 0)
    {
      if (angleDiff < 180_deg)
      {
        return {angle1, angle2};
      }
      else
      {
        return {angle2, angle1};
      }
    }

    if (angle2 > 0 && angle1 < 0)
    {
      if (angleDiff < 180_deg)
      {
        return {angle2, angle1};
      }
      else
      {
        return {angle1, angle2};
      }
    }

    if (angle1 > angle2)
    {
      return {angle1, angle2};
    }
    else
    {
      return {angle2, angle1};
    }
  }

  inline static Angle getLargerMinusSmallerAngleDiff(const Angle& angle1, const Angle& angle2)
  {
    Angle smallValue;
    Angle largeValue;
    if (angle1 < angle2)
    {
      smallValue = angle1;
      largeValue = angle2;
    }
    else
    {
      smallValue = angle2;
      largeValue = angle1;
    }
    return largeValue - smallValue;
  }

  inline static Angle getAngleSmallestDiff(const Angle& angle1, const Angle& angle2)
  {
    const Angle largerMinusSmallerAngleDiff = getLargerMinusSmallerAngleDiff(angle1, angle2);
    if (largerMinusSmallerAngleDiff > 180_deg)
    {
      return 360_deg - largerMinusSmallerAngleDiff;
    }
    return largerMinusSmallerAngleDiff;
  }

  inline static Angle getLeftToRightAngleDiff(const Angle& leftAngle, const Angle& rightAngle)
  {
    if (leftAngle < rightAngle)
    {
      return rightAngle - leftAngle;
    }
    return 360_deg - (leftAngle - rightAngle);
  }

  inline static float getCloseToMiddlePercent(const Angle angle, const Angle rangeLeftAngle, const Angle rangeRightAngle)
  {
    const Angle coneDiff = MathUtils::getLeftToRightAngleDiff(rangeLeftAngle, rangeRightAngle); // todo calculate in cone once
    const Angle halfConeDiff = coneDiff / 2;
    const Angle middle = Angle(rangeLeftAngle - halfConeDiff).normalize();
    const Angle kickAngleToMiddleDiff = MathUtils::getAngleSmallestDiff(angle, middle);
    return std::max(0.f, 1.f - kickAngleToMiddleDiff / halfConeDiff);
  }

  inline static bool isEqual(const float f1, const float f2) { return std::fabs(f1 - f2) < std::numeric_limits<float>::epsilon(); }
};
