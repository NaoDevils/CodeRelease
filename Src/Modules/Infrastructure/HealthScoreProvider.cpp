/**
* @file Modules/Infrastructure/HealthScoreProvider.h
* This file implements a module that provides information about the robot's health.
* @author <a href="mailto:dominik.braemer@tu-dortmund.de">Dominik Brämer</a>
*/


#include "HealthScoreProvider.h"

HealthScoreProvider::HealthScoreProvider()
{
  jointsProbablyBad = false;
}

void HealthScoreProvider::update(HealthScore& healthScore)
{
  healthScore.top = false;
  healthScore.usable = false;
  healthScore.miserable = false;

  bool batteryBroken = theCognitionState.batteryStatus.batteryBroken;
  bool imuBroken = checkImu();
  bool fsrBroken = !theMotionState.fsrStatus.usableLeft && !theMotionState.fsrStatus.usableRight;
  bool cameraBroken = !theCameraInfo.usable && !theCameraInfoUpper.usable; // always true needs changes in CameraProviderV6.cpp

  bool jointsBad = checkJoints();
  bool legsUsable = theMotionState.jointStatus.usableLegs;

  float firstOrderFailure = float(0.5f * jointsBad + 0.5f * !legsUsable);
  float secondOrderFailure = float(batteryBroken + imuBroken + fsrBroken + cameraBroken);
  healthScore.score = std::min(1.f, firstOrderFailure + secondOrderFailure);

  if (healthScore.score > 0.66 && healthScore.score <= 1)
    healthScore.miserable = true;
  if (healthScore.score > 0.33 && healthScore.score <= 0.66)
    healthScore.usable = true;
  if (healthScore.score >= 0 && healthScore.score <= 0.33)
    healthScore.top = true;
}

bool HealthScoreProvider::checkImu()
{
  bool accBroken = std::any_of(theMotionState.imuStatus.accStatus.begin(),
      theMotionState.imuStatus.accStatus.end(),
      [](bool v)
      {
        return !v;
      });
  bool gyroBroken = std::any_of(theMotionState.imuStatus.gyroStatus.begin(),
      theMotionState.imuStatus.gyroStatus.end(),
      [](bool v)
      {
        return !v;
      });
  bool angleBroken = std::any_of(theMotionState.imuStatus.angleStatus.begin(),
      theMotionState.imuStatus.angleStatus.end(),
      [](bool v)
      {
        return !v;
      });
  return accBroken || gyroBroken || angleBroken;
}

bool HealthScoreProvider::checkJoints()
{
  float stumble = theMotionState.walkingStatus.stumble;

  if (stumble > 0.7)
    jointsProbablyBad = true;

  return jointsProbablyBad;
}


MAKE_MODULE(HealthScoreProvider, cognitionInfrastructure)
