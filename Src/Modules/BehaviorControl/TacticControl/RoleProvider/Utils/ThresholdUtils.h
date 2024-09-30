#pragma once

#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"

class ThresholdUtils
{
public:
  static void setThreshholdsLow(PositioningSymbols& positioningSymbols)
  {
    positioningSymbols.thresholdRotation = 5_deg;
    positioningSymbols.thresholdXBack = 20.f;
    positioningSymbols.thresholdXFront = 20.f;
    positioningSymbols.thresholdY = 20.f;
  }

  static void setThreshholdsMedium(PositioningSymbols& positioningSymbols)
  {
    positioningSymbols.thresholdRotation = 15_deg;
    positioningSymbols.thresholdXBack = 75.f;
    positioningSymbols.thresholdXFront = 75.f;
    positioningSymbols.thresholdY = 75.f;
  }

  static void setThreshholdsHeigh(PositioningSymbols& positioningSymbols)
  {
    positioningSymbols.thresholdRotation = 25_deg;
    positioningSymbols.thresholdXBack = 250.f;
    positioningSymbols.thresholdXFront = 250.f;
    positioningSymbols.thresholdY = 250.f;
  }

  static void setThreshholdsExtremeHeigh(PositioningSymbols& positioningSymbols, const float activity)
  {
    positioningSymbols.thresholdRotation = 15_deg + (1 - activity) * 30_deg;
    const float threshold = 500.f + (1 - activity) * 2000.f;
    positioningSymbols.thresholdXBack = threshold;
    positioningSymbols.thresholdXFront = threshold;
    positioningSymbols.thresholdY = threshold;
  }

  static void setThreshholdsCustom(PositioningSymbols& positioningSymbols, Angle threshRot, float threshXBack, float threshXFront, float threshY)
  {
    positioningSymbols.thresholdRotation = threshRot;
    positioningSymbols.thresholdXBack = threshXBack;
    positioningSymbols.thresholdXFront = threshXFront;
    positioningSymbols.thresholdY = threshY;
  }

  static void setThresholdsForAnyKick(PositioningAndKickSymbols& pakSymbols)
  {
    pakSymbols.thresholdXFront = 100.f; // 50.f; // todo changed for gore2023 in commit "higher any kick thresholds"
    pakSymbols.thresholdXBack = 500.f; // 200.f;
    pakSymbols.thresholdY = 300.f; // 150.f;
    pakSymbols.thresholdRotation = 50_deg; // 30_deg;
  }

  static void setThresholdsForDribble(PositioningAndKickSymbols& pakSymbols)
  {
    pakSymbols.thresholdXFront = 200.f;
    pakSymbols.thresholdXBack = 300.f;
    pakSymbols.thresholdY = 100.f;
    pakSymbols.thresholdRotation = 10_deg;
  }

  static void setThresholdsForLongKick(PositioningAndKickSymbols& pakSymbols)
  {
    pakSymbols.thresholdXFront = 20.f; // TODO Should bespecified somewhere in the kick
    pakSymbols.thresholdXBack = 30.f;
    pakSymbols.thresholdY = 15.f;
    pakSymbols.thresholdRotation = 5_deg;
  }
};
