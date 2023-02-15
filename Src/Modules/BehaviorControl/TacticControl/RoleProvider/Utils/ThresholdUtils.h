#pragma once

#include <Representations/BehaviorControl/PositioningSymbols.h>

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

  static void setThreshholdsCustom(PositioningSymbols& positioningSymbols, Angle threshRot, float threshXBack, float threshXFront, float threshY)
  {
    positioningSymbols.thresholdRotation = threshRot;
    positioningSymbols.thresholdXBack = threshXBack;
    positioningSymbols.thresholdXFront = threshXFront;
    positioningSymbols.thresholdY = threshY;
  }

  static void setThresholdsForAnyKick(Ballchaser& ballchaser)
  {
    ballchaser.thresholdXFront = 50.f;
    ballchaser.thresholdXBack = 200.f;
    ballchaser.thresholdY = 150.f;
    ballchaser.thresholdRotation = 30_deg;
  }

  static void setThresholdsForLongKick(Ballchaser& ballchaser)
  {
    ballchaser.thresholdXFront = 20.f; // TODO Should bespecified somewhere in the kick
    ballchaser.thresholdXBack = 30.f;
    ballchaser.thresholdY = 15.f;
    ballchaser.thresholdRotation = 5_deg;
  }
};
