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

OracledWorldModelProvider::OracledWorldModelProvider() : lastBallModelComputation(0), lastMultipleBallModelComputation(0), lastRobotPoseComputation(0)
{
  lastBallPositions.push_back(Vector2f::Zero());
}

void OracledWorldModelProvider::computeRobotPose()
{
  if (lastRobotPoseComputation == theFrameInfo.time)
    return;
  DRAW_ROBOT_POSE("module:OracledWorldModelProvider:realRobotPose", theGroundTruthWorldState.ownPose, ColorRGBA::magenta);
  theRobotPose = theGroundTruthWorldState.ownPose + robotPoseOffset;
  theRobotPose.sideConfidenceState = SideConfidence::ConfidenceState::CONFIDENT;
  theRobotPose.validity = 1.f;
  lastRobotPoseComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::computeBallModel()
{
  if (lastBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  computeRobotPose();
  Vector2f ballPosition = theGroundTruthWorldState.balls[0];

  Vector2f velocity((ballPosition - lastBallPositions[0]) / float(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) * 1000.f);
  theBallModel.estimate.position = theRobotPose.inverse() * ballPosition;
  theBallModel.estimate.velocity = velocity.rotate(-theRobotPose.rotation);
  theBallModel.lastPerception = theBallModel.estimate.position;
  theBallModel.timeWhenLastSeen = theFrameInfo.time;
  theBallModel.timeWhenDisappeared = theFrameInfo.time;
  theBallModel.validity = 1.f;

  lastBallPositions[0] = ballPosition;
  lastBallModelComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::computeMultipleBallModel()
{
  if (lastMultipleBallModelComputation == theFrameInfo.time || theGroundTruthWorldState.balls.size() == 0)
    return;
  computeRobotPose();

  if (lastBallPositions.size() != theGroundTruthWorldState.balls.size())
    lastBallPositions.resize(theGroundTruthWorldState.balls.size(), Vector2f::Zero());
  if (theMultipleBallModel.ballModels.size() != theGroundTruthWorldState.balls.size())
    theMultipleBallModel.ballModels.resize(theGroundTruthWorldState.balls.size());

  for (size_t i = 0; i < theGroundTruthWorldState.balls.size(); i++)
  {
    Vector2f ballPosition = theGroundTruthWorldState.balls[i];
    Vector2f velocity((ballPosition - lastBallPositions[i]) / float(theFrameInfo.getTimeSince(theMultipleBallModel.ballModels[i].timeWhenLastSeen)) * 1000.f);
    theMultipleBallModel.ballModels[i].estimate.position = theRobotPose.inverse() * ballPosition;
    theMultipleBallModel.ballModels[i].estimate.velocity = velocity.rotate(-theRobotPose.rotation);
    theMultipleBallModel.ballModels[i].lastPerception = theMultipleBallModel.ballModels[i].estimate.position;
    theMultipleBallModel.ballModels[i].timeWhenLastSeen = theFrameInfo.time;
    theMultipleBallModel.ballModels[i].timeWhenDisappeared = theFrameInfo.time;
    theMultipleBallModel.ballModels[i].validity = 1.f;
    lastBallPositions[i] = ballPosition;
  }
  lastMultipleBallModelComputation = theFrameInfo.time;
}

void OracledWorldModelProvider::update(BallModel& ballModel)
{
  computeBallModel();
  ballModel = theBallModel;
}

void OracledWorldModelProvider::update(MultipleBallModel& multipleBallModel)
{
  computeMultipleBallModel();
  multipleBallModel = theMultipleBallModel;
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

void OracledWorldModelProvider::update(GroundTruthMultipleBallModel& groundTruthMultipleBallModel)
{
  computeMultipleBallModel();

  if (groundTruthMultipleBallModel.ballModels.size() != theMultipleBallModel.ballModels.size())
  {
    groundTruthMultipleBallModel.ballModels.resize(theMultipleBallModel.ballModels.size());
  }

  for (size_t i = 0; i < theMultipleBallModel.ballModels.size(); i++)
  {
    groundTruthMultipleBallModel.ballModels[i].estimate = theMultipleBallModel.ballModels[i].estimate;
    groundTruthMultipleBallModel.ballModels[i].lastPerception = theMultipleBallModel.ballModels[i].lastPerception;
    groundTruthMultipleBallModel.ballModels[i].timeWhenLastSeen = theMultipleBallModel.ballModels[i].timeWhenLastSeen;
    groundTruthMultipleBallModel.ballModels[i].timeWhenDisappeared = theMultipleBallModel.ballModels[i].timeWhenDisappeared;
    groundTruthMultipleBallModel.ballModels[i].validity = theMultipleBallModel.ballModels[i].validity;
  }
}

void OracledWorldModelProvider::update(RobotMap& robotMap)
{
  computeRobotPose();
  robotMap.robots.clear();
  if (!Global::settingsExist())
    return;

  // Simulation scene should only use blue and red for now
  //ASSERT(Global::getSettings().teamColor == Settings::blue || Global::getSettings().teamColor == Settings::red);

  const bool teammate = Global::getSettings().teamNumber == 1;
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
