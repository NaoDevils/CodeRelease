/**
* @file Modules/Infrastructure/OracledWorldModelProvider.h
*
* This file implements a module that provides models based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(OracledWorldModelProvider,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  PROVIDES(BallModel),
  PROVIDES(MultipleBallModel),
  PROVIDES(GroundTruthBallModel),
  PROVIDES(GroundTruthMultipleBallModel),
  PROVIDES(RobotMap),
  PROVIDES(RobotPose),
  PROVIDES(GroundTruthRobotPose),
  LOADS_PARAMETERS(,
    (Pose2f) robotPoseOffset, /**< Offset that will be added to the robot pose. Useful for testing */
    (float) obstacleModelMaxDistance /**< Only obstacles (players, goalposts) will be entered in the obstacle model if their distance to the robot is closer that this parameter value */
  )
);

/**
* @class OracledWorldModelProvider
* A module that provides several models
*/
class OracledWorldModelProvider : public OracledWorldModelProviderBase
{
public:
  /** Constructor*/
  OracledWorldModelProvider();

private:
  /** The function that actually computes the ball model*/
  void computeBallModel();
  /** The function that actually computes the multiple ball model*/
  void computeMultipleBallModel();

  /** The function that actually computes the robot pose*/
  void computeRobotPose();

  /** One main function, might be called every cycle
  * @param ballModel The data struct to be filled
  */
  void update(BallModel& ballModel);

  /** One main function, might be called every cycle
  * @param ballModel The data struct to be filled
  */
  void update(MultipleBallModel& multipleBallModel);

  /** One main function, might be called every cycle
  * @param groundTruthBallModel The data struct to be filled
  */
  void update(GroundTruthBallModel& groundTruthBallModel);

  /** One main function, might be called every cycle
* @param groundTruthBallModel The data struct to be filled
*/
  void update(GroundTruthMultipleBallModel& groundTruthMultipleBallModel);

  /** One main function, might be called every cycle
  * @param robotMap The data struct to be filled
  */
  void update(RobotMap& robotMap);

  /** One main function, might be called every cycle
  * @param robotPose The data struct to be filled
  */
  void update(RobotPose& robotPose);

  /** One main function, might be called every cycle
  * @param groundTruthRobotPose The data struct to be filled
  */
  void update(GroundTruthRobotPose& groundTruthRobotPose);

  /** Converts ground truth player data to an robot map entry
  * @param player A player
  * @param robotMap The robot map to which the player will be added
  * @param isTeammate Whether a player is in own team or not
  */
  void playerToRobotMapEntry(const GroundTruthWorldState::GroundTruthPlayer& player, RobotMap& robotMap, const bool isTeammate) const;

  unsigned int lastBallModelComputation; /**< Time of last ball model computation*/
  unsigned int lastMultipleBallModelComputation; /**< Time of last ball model computation*/
  unsigned int lastRobotPoseComputation; /**< Time of last robot pose computation*/
  std::vector<Vector2f> lastBallPositions; /**< The ball positions after the last computation*/
  BallModel theBallModel; /**< The current ball model*/
  MultipleBallModel theMultipleBallModel; /**< The current ball model*/
  RobotPose theRobotPose; /**< The current robot pose*/
};
