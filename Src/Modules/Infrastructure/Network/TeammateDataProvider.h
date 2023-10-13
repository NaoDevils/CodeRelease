/**
 * @file Modules/Infrastructure/TeammateDataProvider.h
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Module/Module.h"


MODULE(TeammateDataProvider,
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeamCommInput),
  REQUIRES(TimeOffsets),
  PROVIDES(TeammateData),
  DEFINES_PARAMETERS(,
    (int)(2500) badWifiTimeout /**< Time in ms after which a package is considered too old and discarded. */
  )
);

/**
* @class TeammateDataProvider
* A module that provides information about the teammates
*/
class TeammateDataProvider : public TeammateDataProviderBase
{
private:
  float wlanQuality = 1.f; /**< [0..1] Quality of wifi */
  size_t lastNoOfTeammates = 0;

  /** The main function, called every cycle
   * @param teammateData The data struct to be filled
   */
  void update(TeammateData& teammateData);

  /**
   * Check if package is too old (slow wireless).
   * @param message Received teammate.
   * @return If package is new enough.
  */
  bool isLatencyOkay(const TeammateReceived& teammate);
};
