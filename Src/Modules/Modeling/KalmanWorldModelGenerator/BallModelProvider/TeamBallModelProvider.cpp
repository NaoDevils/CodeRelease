/**
 * \file TeamBallModelProvider.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 */

#include "TeamBallModelProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"


// ---------- TeamBallModelProvider methods ----------

void TeamBallModelProvider::initialize(TeamBallModelParameters& teamParameters)
{
  // Save TeamBallModel parameters.
  m_teamParameters = teamParameters;

  // Set initial (invalid) values of TeamBallModel.
  m_teamBallModel.isValid = false;
  m_teamBallModel.isLocalBallModel = false;
  m_teamBallModel.position = Vector2f::Zero();
  m_teamBallModel.velocity = Vector2f::Zero();
  m_teamBallModel.timeWhenLastValid = 0;
  m_teamBallModel.validity = 0.f;
}

void TeamBallModelProvider::generateTeamBallModel(
    TeamBallModel& teamBallModel, LocalMultipleBallModel& localMultipleBallModel, RemoteMultipleBallModel& remoteMultipleBallModel, const RobotPose& theRobotPose, const FrameInfo& theFrameInfo)
{
  // --- Debug methods ---

  // Allow modification of some parameters in SimRobot.
  m_initModify();


  // --- TeamBallModelProvider methods ---

  // Decide whether to use local or remote ball model based on validity.
  float localValidity = 0.f;
  float remoteValidity = 0.f;
  const KalmanPositionHypothesis* bestLocalHypothesis = localMultipleBallModel.bestHypothesis();
  const KalmanPositionHypothesis* bestRemoteHypothesis = remoteMultipleBallModel.bestHypothesis();
  if (bestLocalHypothesis != nullptr)
    localValidity = bestLocalHypothesis->validity;
  if (bestRemoteHypothesis != nullptr)
    remoteValidity = bestRemoteHypothesis->validity;

  if (localValidity >= m_teamParameters.minValidityForLocalBallModel)
  {
    // Set valid.
    m_teamBallModel.isValid = true;
    m_teamBallModel.timeWhenLastValid = theFrameInfo.time;

    // Use local ball model.
    m_teamBallModel.isLocalBallModel = true;
    generateTeamBallModelFromBallHypothesis(m_teamBallModel, bestLocalHypothesis, &theRobotPose);
  }
  else if (remoteValidity >= m_teamParameters.minValidityForRemoteBallModel)
  {
    // Set valid.
    m_teamBallModel.isValid = true;
    m_teamBallModel.timeWhenLastValid = theFrameInfo.time;

    // Use remote ball model.
    m_teamBallModel.isLocalBallModel = false;
    generateTeamBallModelFromBallHypothesis(m_teamBallModel, bestRemoteHypothesis, nullptr);
  }
  else
  {
    // No valid ball model.
    m_teamBallModel.isValid = false;

    // Although there is no valid ball model, update the state of the \c teamBallModel
    // with the last used model (local or remote).
    if (m_teamBallModel.isLocalBallModel)
      generateTeamBallModelFromBallHypothesis(m_teamBallModel, bestLocalHypothesis, &theRobotPose);
    else
      generateTeamBallModelFromBallHypothesis(m_teamBallModel, bestRemoteHypothesis, nullptr);
  }

  // Write private member m_teamBallModel to output parameter.
  teamBallModel = m_teamBallModel;


  // --- Debug methods ---

  // Draw debug plots.
  m_plot(localValidity, remoteValidity, localMultipleBallModel.size(), remoteMultipleBallModel.size(), bestLocalHypothesis, bestRemoteHypothesis);
}

void TeamBallModelProvider::generateTeamBallModelFromBallHypothesis(TeamBallModel& teamBallModel, const KalmanPositionHypothesis* hypothesis, const RobotPose* theRobotPose)
{
  if (hypothesis != nullptr)
  {
    if (theRobotPose != nullptr)
    {
      // Transform position and velocity of ball hypothesis from relative robot
      // coordinates to global field coordinates.
      teamBallModel.position = Transformation::robotToField(*theRobotPose, hypothesis->kalman.position());
      teamBallModel.velocity = Transformation::robotToFieldVelocity(*theRobotPose, hypothesis->kalman.velocity());
    }
    else
    {
      teamBallModel.position = hypothesis->kalman.position();
      teamBallModel.velocity = hypothesis->kalman.velocity();
    }
    teamBallModel.validity = hypothesis->validity;
  }
  else
  {
    // No valid ball model.
    teamBallModel.isValid = false;
    teamBallModel.validity = 0.f;
  }
}


// ---------- Debug methods ----------

void TeamBallModelProvider::m_initModify()
{
  MODIFY("module:BallModelProvider:teamParameters", m_teamParameters);
}

void TeamBallModelProvider::m_plot(
    float localValidity, float remoteValidity, size_t localHypothesesCount, size_t remoteHypothesesCount, const KalmanPositionHypothesis* bestLocalHypothesis, const KalmanPositionHypothesis* bestRemoteHypothesis) const
{
  /*
  vp teamBallModelValidity 300 0 1 validity s 0.033
  vpd teamBallModelValidity module:BallModelProvider:localValidity red
  vpd teamBallModelValidity module:BallModelProvider:localValidityMin orange
  vpd teamBallModelValidity module:BallModelProvider:remoteValidity black
  vpd teamBallModelValidity module:BallModelProvider:remoteValidityMin gray
   */

  // Number of hypotheses:
  DECLARE_PLOT("module:BallModelProvider:localNumberOfHypotheses");
  DECLARE_PLOT("module:BallModelProvider:remoteNumberOfHypotheses");
  PLOT("module:BallModelProvider:localNumberOfHypotheses", localHypothesesCount);
  PLOT("module:BallModelProvider:remoteNumberOfHypotheses", remoteHypothesesCount);

  // Validities:
  DECLARE_PLOT("module:BallModelProvider:localValidity");
  DECLARE_PLOT("module:BallModelProvider:localValidityMin");
  DECLARE_PLOT("module:BallModelProvider:remoteValidity");
  DECLARE_PLOT("module:BallModelProvider:remoteValidityMin");
  PLOT("module:BallModelProvider:localValidity", localValidity);
  PLOT("module:BallModelProvider:localValidityMin", m_teamParameters.minValidityForLocalBallModel);
  PLOT("module:BallModelProvider:remoteValidity", remoteValidity);
  PLOT("module:BallModelProvider:remoteValidityMin", m_teamParameters.minValidityForRemoteBallModel);

  // Percepts per second:
  float localPPS = 0, remotePPS = 0;
  float localMeanPerceptValidity = 0.f, remoteMeanPerceptValidity = 0.f;
  if (bestLocalHypothesis != nullptr)
  {
    localPPS = bestLocalHypothesis->perceptsPerSecond();
    localMeanPerceptValidity = bestLocalHypothesis->meanPerceptValidity();
  }
  if (bestRemoteHypothesis != nullptr)
  {
    remotePPS = bestRemoteHypothesis->perceptsPerSecond();
    remoteMeanPerceptValidity = bestRemoteHypothesis->meanPerceptValidity();
  }

  DECLARE_PLOT("module:BallModelProvider:localPerceptsPerSecond");
  DECLARE_PLOT("module:BallModelProvider:remotePerceptsPerSecond");
  PLOT("module:BallModelProvider:localPerceptsPerSecond", localPPS);
  PLOT("module:BallModelProvider:remotePerceptsPerSecond", remotePPS);
  DECLARE_PLOT("module:BallModelProvider:localValidityPPS");
  DECLARE_PLOT("module:BallModelProvider:remoteValidityPPS");
  PLOT("module:BallModelProvider:localValidityPPS", localMeanPerceptValidity * static_cast<float>(localPPS));
  PLOT("module:BallModelProvider:remoteValidityPPS", remoteMeanPerceptValidity * static_cast<float>(remotePPS));
}
