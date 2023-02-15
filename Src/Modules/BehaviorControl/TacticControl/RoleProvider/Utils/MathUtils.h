#pragma once

class MathUtils
{
public:
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

  /**
   * @return a vector that is not normalized
   */
  inline static Vector2f angleToVector(const Angle angle) { return {(float)std::cos(angle), (float)std::sin(angle)}; }

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

  inline static float getAngleDiff(const Angle& angle1, const Angle& angle2, const bool hysteresis)
  {
    return std::fabs(Angle::normalize(angle1 - angle2)) * (hysteresis ? 0.9f : 1.f);
  }

  inline static bool isEqual(const float f1, const float f2) { return std::fabs(f1 - f2) < std::numeric_limits<float>::epsilon(); }
};
