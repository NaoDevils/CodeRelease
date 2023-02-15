/**
 * @file Modules/Infrastructure/TeammateDataProvider.h
 * This file declares a temporary wrapper from the old to the new TeammateData structure.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Module/Module.h"


MODULE(TeammateDataProvider,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(TeamCommInput),
  REQUIRES(TimeOffsets),
  USES(MotionRequest),
  USES(RobotPose), // to compare team model to own
  USES(BallModel), // to compare team model to own
  PROVIDES(TeammateData),
  DEFINES_PARAMETERS(,
    (int)(1000) sendInterval, /** <  Time in ms between two messages that are sent to the teammates */
    (int)(4000) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
    (int)(2500) badWifiTimeout, /**< Time in ms after which a package is considered too old and discarded. */
    (bool)(false) useMixedTeamBallModel,
    (float)(-0.5f) minSanityForTeammates
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
  unsigned messageCounts[MAX_NUM_PLAYERS] = {0}; /**< Message Counter for all players for debugging. */

  /** The main function, called every cycle
   * @param teammateData The data struct to be filled
   */
  void update(TeammateData& teammateData);

  /**
   * Check if package is too old (slow wireless).
   * @param message Received package.
   * @return If package is new enough.
  */
  bool isLatencyOkay(const TeamCommData& message);

  /**
   * Returns the representation of the robot that has the "robotNumber".
   * If the robot does not exist yet, the object will be created.
   * @param message The message of the robot to look for.
   */
  static Teammate& getTeammate(TeammateData& teammateData, const TeamCommData& message);

  /**
   * The method is called for every incoming team message and fills a given team mate.
   * @param message An interface to read the message from the queue.
   * @param teammate The team mate to be filled.
   */
  void fillTeammate(InMessage& message, Teammate& teammate) const;

  /**
   * Fill Teammate using TeamCommData.
   * @param message Team communication data
   * @param teammate Team mate
   */
  void fillTeammate(const TeamCommData& message, Teammate& teammate) const;
};
