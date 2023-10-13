#pragma once

class HysteresisUtils
{
public:
  static bool compareAbsolutes(const bool current, const float returnTrueIfSmall, const float returnTrueIfBig, const float addToOneMultiplier)
  {
    if (current)
      return returnTrueIfSmall < returnTrueIfBig * (1 + addToOneMultiplier);
    else
      return returnTrueIfSmall * (1 + addToOneMultiplier) < returnTrueIfBig;
  }

  static bool comparePossibleNegatives(const bool current, const float returnTrueIfSmall, const float returnTrueIfBig, const float addOrSubtractFromBig)
  {
    if (current)
      return returnTrueIfSmall < returnTrueIfBig + addOrSubtractFromBig;
    else
      return returnTrueIfSmall < returnTrueIfBig - addOrSubtractFromBig;
  }

  static bool comparePossibleNegatives(const bool current, const float returnTrueIfSmall, const float returnTrueIfBig, const float addIfCurrentIsFalse, const float addIfCurrentIsTrue)
  {
    if (current)
      return returnTrueIfSmall < returnTrueIfBig + addIfCurrentIsTrue;
    else
      return returnTrueIfSmall + addIfCurrentIsFalse < returnTrueIfBig;
  }

  static float makeBigger(const float value, const float addition, const float multiplierOrDivisor)
  {
    if (value < 0)
      return addition + value / multiplierOrDivisor;
    else
      return addition + value * multiplierOrDivisor;
  }
};
