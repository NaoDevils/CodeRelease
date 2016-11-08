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
  m_teamBallModel.isLocalBallModel = true;
  m_teamBallModel.position = Vector2f::Zero();
  m_teamBallModel.velocity = Vector2f::Zero();
  m_teamBallModel.timeWhenLastValid = 0;
  m_teamBallModel.validity = 0.f;
}

void TeamBallModelProvider::generateTeamBallModel(TeamBallModel& teamBallModel,
  const MultipleBallModel& localMultipleBallModel, const RemoteMultipleBallModel& remoteMultipleBallModel,
  const RobotPose& theRobotPose, const FrameInfo& theFrameInfo)
{
  // --- Debug methods ---

  // Allow modification of some parameters in SimRobot.
  m_initModify();


  // --- TeamBallModelProvider methods ---

  // Decide whether to use local or remote ball model based on validity.
  float localValidity = 0.f;
  float remoteValidity = 0.f;
  const BallHypothesis* bestLocalHypothesis = localMultipleBallModel.bestHypothesis();
  const BallHypothesis* bestRemoteHypothesis = remoteMultipleBallModel.bestHypothesis();
  if (bestLocalHypothesis != nullptr)
    localValidity = bestLocalHypothesis->validity;
  if (bestRemoteHypothesis != nullptr)
    remoteValidity = bestRemoteHypothesis->validity;
  
  if (localValidity >= m_teamParameters.minValidityForLocalBallModel)
  {
    // Use local ball model.
    m_teamBallModel.isLocalBallModel = true;
    // Transform position and velocity of ball estimate from relative robot 
    // coordinates to global field coordinates.
    m_teamBallModel.position = Transformation::robotToField(theRobotPose, bestLocalHypothesis->kalman.position());
    m_teamBallModel.velocity = Transformation::robotToFieldVelocity(theRobotPose, bestLocalHypothesis->kalman.velocity());
    // Set validity
    m_teamBallModel.isValid = true;
    m_teamBallModel.timeWhenLastValid = theFrameInfo.time;
    m_teamBallModel.validity = localValidity;
  }
  else if (remoteValidity >= m_teamParameters.minValidityForRemoteBallModel)
  {
    // Use remote ball model.
    m_teamBallModel.isLocalBallModel = false;
    m_teamBallModel.position = bestRemoteHypothesis->kalman.position();
    m_teamBallModel.velocity = bestRemoteHypothesis->kalman.velocity();
    // Set validity
    m_teamBallModel.isValid = true;
    m_teamBallModel.timeWhenLastValid = theFrameInfo.time;
    m_teamBallModel.validity = remoteValidity;
  }
  else
  {
    // No valid ball model.
    m_teamBallModel.isValid = false;
    m_teamBallModel.validity = 0.f;
  }
  
  // Write private member m_teamBallModel to output parameter.
  teamBallModel = m_teamBallModel;
  
  
  // --- Debug methods ---
  
  unsigned int localPPS = 0, remotePPS = 0;
  if (bestLocalHypothesis != nullptr)
    localPPS = bestLocalHypothesis->perceptsPerSecond();
  if (bestRemoteHypothesis != nullptr)
    remotePPS = bestRemoteHypothesis->perceptsPerSecond();
  
  // Draw debug plots.
  m_plot(localValidity,
       remoteValidity,
       static_cast<unsigned int>(localMultipleBallModel.numberOfHypotheses()),
       static_cast<unsigned int>(remoteMultipleBallModel.numberOfHypotheses()),
       localPPS,
       remotePPS);
}


// ---------- Debug methods ----------

void TeamBallModelProvider::m_initModify()
{
  MODIFY("module:BallModelProvider:teamParameters", m_teamParameters);
}

void TeamBallModelProvider::m_plot(float localValidity, float remoteValidity,
                                   unsigned int localHypothesesCount, unsigned int remoteHypothesesCount,
                                   unsigned int localPPS, unsigned int remotePPS)
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
  DECLARE_PLOT("module:BallModelProvider:localPerceptsPerSecond");
  DECLARE_PLOT("module:BallModelProvider:remotePerceptsPerSecond");
  PLOT("module:BallModelProvider:localPerceptsPerSecond", localPPS);
  PLOT("module:BallModelProvider:remotePerceptsPerSecond", remotePPS);
}