/**
* @file Modules/Infrastructure/HealthScoreProvider.h
* This file declares a module that provides information about the robot's health.
* @author <a href="mailto:dominik.braemer@tu-dortmund.de">Dominik Brämer</a>
*/

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Perception/CognitionState.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/Infrastructure/CameraInfo.h"

#include "Representations/Infrastructure/HealthScore.h"


MODULE(HealthScoreProvider,
  REQUIRES(MotionState),
  REQUIRES(CognitionState),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),

  PROVIDES(HealthScore)
);

/**
* @class HealthScoreProvider
* A module that provides information about the robot's health
*/
class HealthScoreProvider : public HealthScoreProviderBase
{
public:
  /** Constructor. */
  HealthScoreProvider();

private:
  /** The main function, called every cycle
  * @param healthScore The data struct to be filled
  */
  void update(HealthScore& healthScore);

  bool checkImu();
  bool checkJoints();

  bool jointsProbablyBad = false;
};
