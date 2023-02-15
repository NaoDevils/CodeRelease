#pragma once

class HysterUtils // Hysteresis
{
public:
  static bool compareAbsolutes(const bool current, const float trueIfSmall, const float trueIfBig, const float addToOneMultiplier)
  {
    if (current)
      return trueIfSmall < trueIfBig * (1 + addToOneMultiplier);
    else
      return trueIfSmall * (1 + addToOneMultiplier) < trueIfBig;
  }

  static bool comparePossibleNegatives(const bool current, const float trueIfSmall, const float trueIfBig, const float addOrSubtractFromBig)
  {
    if (current)
      return trueIfSmall < trueIfBig + addOrSubtractFromBig;
    else
      return trueIfSmall < trueIfBig - addOrSubtractFromBig;
  }

  static bool comparePossibleNegatives(const bool current, const float trueIfSmall, const float trueIfBig, const float addFalse, const float addTrue)
  {
    if (current)
      return trueIfSmall < trueIfBig + addTrue;
    else
      return trueIfSmall + addFalse < trueIfBig;
  }

  static float makeBigger(const float value, const float addition, const float multiplierOrDivisor)
  {
    if (value < 0)
      return addition + value / multiplierOrDivisor;
    else
      return addition + value * multiplierOrDivisor;
  }
};
