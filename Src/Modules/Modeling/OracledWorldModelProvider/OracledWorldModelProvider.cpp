/**
* @file Modules/Infrastructure/OracledWorldModelProvider.h
*
* This file implements a module that provides models based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#include "OracledWorldModelProvider.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

OracledWorldModelProvider::OracledWorldModelProvider():
  lastBallModelComputation(0), lastRobotPoseComputation(0)
{}

void OracledWorldModelProvider::computeRobotPose()
{
  if(lastRobotPoseComputation == theFrameInfo.time)
    return;
  DRAW_ROBOT_POSE("module:OracledWorldModelProvider:realRobotPose", theGroundTruthWorldState.ownPose, ColorRGBA::magenta);
  theRobotPose = theGroundTruthWorldState.ownPose + robotPoseOffset;
  theRobotPose.validity = 1.f;
  lastRobotPoseComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::computeBallModel()
{
  if(lastBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  computeRobotPose();
  Vector2f ballPosition = theGroundTruthWorldState.balls[0];

  Vector2f velocity((ballPosition - lastBallPosition) / float(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) * 1000.f);
  theBallModel.estimate.position = theRobotPose.inverse() * ballPosition;
  theBallModel.estimate.velocity = velocity.rotate(-theRobotPose.rotation);
  theBallModel.lastPerception = theBallModel.estimate.position;
  theBallModel.timeWhenLastSeen = theFrameInfo.time;
  theBallModel.timeWhenDisappeared = theFrameInfo.time;
  theBallModel.validity = 1.f;

  lastBallPosition = ballPosition;
  lastBallModelComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::update(BallModel& ballModel)
{
  computeBallModel();
  ballModel = theBallModel;
}

void OracledWorldModelProvider::update(GroundTruthBallModel& groundTruthBallModel)
{
  computeBallModel();
  groundTruthBallModel.lastPerception = theBallModel.lastPerception;
  groundTruthBallModel.estimate = theBallModel.estimate;
  groundTruthBallModel.timeWhenDisappeared = theBallModel.timeWhenDisappeared;
  groundTruthBallModel.timeWhenLastSeen = theBallModel.timeWhenLastSeen;
  groundTruthBallModel.validity = theBallModel.validity;
}

void OracledWorldModelProvider::update(RobotMap& robotMap)
{
  computeRobotPose();
  robotMap.robots.clear();
  if (!Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);

  const bool teammate = Global::getSettings().teamColor == Settings::blue;
  for (unsigned int i = 0; i < theGroundTruthWorldState.bluePlayers.size(); ++i)
    playerToRobotMapEntry(theGroundTruthWorldState.bluePlayers[i], robotMap, teammate);
  for (unsigned int i = 0; i < theGroundTruthWorldState.redPlayers.size(); ++i)
    playerToRobotMapEntry(theGroundTruthWorldState.redPlayers[i], robotMap, !teammate);
}

void OracledWorldModelProvider::playerToRobotMapEntry(const GroundTruthWorldState::GroundTruthPlayer& player, RobotMap& robotMap, const bool isTeammate) const
{
  Vector2f center(theRobotPose.inverse() * player.pose.translation);
  if (center.squaredNorm() >= sqr(obstacleModelMaxDistance))
    return;
  RobotMapEntry re;
  re.pose = Pose2f(0, player.pose.translation);
  re.robotType = isTeammate ? RobotEstimate::teammateRobot : RobotEstimate::opponentRobot;
  robotMap.robots.emplace_back(re);
}

void OracledWorldModelProvider::update(RobotPose& robotPose)
{
  DECLARE_DEBUG_DRAWING("module:OracledWorldModelProvider:realRobotPose", "drawingOnField");
  computeRobotPose();
  robotPose = theRobotPose;
}

void OracledWorldModelProvider::update(GroundTruthRobotPose& groundTruthRobotPose)
{
  computeRobotPose();
  groundTruthRobotPose.translation = theRobotPose.translation;
  groundTruthRobotPose.rotation = theRobotPose.rotation;
  groundTruthRobotPose.validity = theRobotPose.validity;
  groundTruthRobotPose.timestamp = theFrameInfo.time;
}

MAKE_MODULE(OracledWorldModelProvider, cognitionInfrastructure)
