/**
 * \file BallModelProvider.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 */

#include "BallModelProvider.h"

MAKE_MODULE(BallModelProvider, modeling)


// MARK: update methods for representations

void BallModelProvider::update(BallModel& ballModel)
{
  execute();
  ballModel = m_localBallModel;
}

void BallModelProvider::update(RemoteBallModel& remoteBallModel)
{
  execute();
  remoteBallModel = m_remoteBallModel;
}

void BallModelProvider::update(TeamBallModel& teamBallModel)
{
  execute();
  m_teamBallModelProvider.generateTeamBallModel(
    teamBallModel, m_localMultipleBallModel, m_remoteMultipleBallModel,
    theRobotPose, theFrameInfo);
}


// MARK: BallModelProvider methods

void BallModelProvider::initialize()
{
  m_lastGameState = STATE_INITIAL;
  m_lastPenaltyState = PENALTY_NONE;
  
  // Local ball model.
  m_localBallModel.timeWhenLastSeenByTeamMate = 0;
  
  // Remote ball model.
  m_remoteBallModel.timeWhenLastSeen = 0;
  m_remoteTimeSinceBallSeen = std::vector<unsigned>(7); // one value per teammate (index = player number)
  for (size_t i = 0; i < m_remoteTimeSinceBallSeen.size(); i++)
    m_remoteTimeSinceBallSeen[i] = 0;

  // Team ball model.
  m_teamBallModelProvider.initialize(teamParameters);
  
  // Set parameters for updating the index of the best ball hypothesis.
  m_localMultipleBallModel.setParametersForUpdateBestHypothesis(
    parameters.local.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
    parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis,  // and at least this amount of sensor updates.
    parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);
  m_remoteMultipleBallModel.setParametersForUpdateBestHypothesis(
    parameters.remote.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
    parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis,   // and at least this amount of sensor updates.
    parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);
}

void BallModelProvider::execute()
{
  // Only update once per step.
  if (m_lastTimeStamp == theFrameInfo.time) {
    return;
  }


  // --- Debug methods ---

  // Allow modification of some parameters in SimRobot.
  initModify();
  // Init debug drawings.
  initDebugDrawing();


  // --- Filter ---
  
  // Add a ball hypothesis to the field center at the transition from game state set to playing.
  handleGameState();
  // Increase velocity uncertainty if a kick is executed;
  handleKick();
  
  // Perform prediction step.
  motionUpdate();
  // Perform correction step.
  sensorUpdate();
  

  // Remove hypotheses with too small validities and those which are too similar 
  // to the best hypothesis.
  if (parameters.CleanUpHypotheses_outsideField)
  {
    m_localMultipleBallModel.cleanUpHypothesesOutsideField(
      theFieldDimensions, theRobotPose,
      parameters.CleanUpHypotheses_fieldBorderThreshold);
    m_remoteMultipleBallModel.cleanUpHypothesesOutsideField(
      theFieldDimensions, theRobotPose,
      parameters.CleanUpHypotheses_fieldBorderThreshold);
  }
  m_localMultipleBallModel.cleanUpHypothesesLowValidity(
    parameters.CleanUpHypotheses_belowValidity, true);
  m_localMultipleBallModel.cleanUpHypothesesSimilarToBestOne(
    parameters.local.Hypotheses_minDistanceForNewHypothesis,
    parameters.CleanUpHypotheses_minAngleForKeepingHypothesis);
  
  m_remoteMultipleBallModel.cleanUpHypothesesLowValidity(
    parameters.CleanUpHypotheses_belowValidity, true);
  m_remoteMultipleBallModel.cleanUpHypothesesSimilarToBestOne(
    parameters.remote.Hypotheses_minDistanceForNewHypothesis,
    parameters.CleanUpHypotheses_minAngleForKeepingHypothesis);
  
  // Create standard BallModel and RemoteBallModel from multiple ball models.
  generateLocalBallModel();
  generateRemoteBallModel();

  // --- Debug methods ---

  // Do debug drawings.
  draw();


  // --- Save current state for next iteration ---
  m_lastOdometryData = theOdometryData;
  m_lastTimeStamp = theFrameInfo.time;
}

void BallModelProvider::motionUpdate()
{
  // Calculate odometry change since last iteration.
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;

  // Apply odometry offset to all local ball hypotheses.
  m_localMultipleBallModel.removeOdometry(odometryOffset);

  // ----- Perform prediction step on all (local and remote) hypotheses  -----
  m_localMultipleBallModel.motionUpdate(theFrameInfo.time, theFieldDimensions.ballFriction); // ballFriction < 0
  m_remoteMultipleBallModel.motionUpdate(theFrameInfo.time, theFieldDimensions.ballFriction); // ballFriction < 0
}

void BallModelProvider::sensorUpdate()
{
  // ----- Perform correction step on all local hypotheses -----
  sensorUpdateLocal();

  // ----- Perform correction step on all remote hypotheses -----
  sensorUpdateRemote();
}

void BallModelProvider::sensorUpdateLocal()
{
  if (theMultipleBallPercept.balls.size() > 0)
  {
    // Use multiple ball percept.
    for (const BallPercept& ball : theMultipleBallPercept.balls)
      sensorUpdateLocalSingle(ball);
  }
  else
  {
    // Multiple ball percept is not filled. Use default ball model.
    sensorUpdateLocalSingle(theBallPercept);
  }
  
  // Update validity of all hypotheses.
  m_localMultipleBallModel.updateValidity(
    parameters.local.Validity_maxPerceptsPerSecond,
    parameters.local.goodValidity(),
    parameters.local.Validity_weightOfPreviousValidity,
    parameters.local.Validity_weightOfPreviousValidity_goodHypotheses);
}

void BallModelProvider::sensorUpdateLocalSingle(const BallPercept& ball)
{
  // Sensor update (correction of kalman filters) cannot be done for local ball
  // hypotheses, if the ball was not perceived.
  if (ball.status == BallPercept::seen)
  {
    // Run sensor update of each local ball hypothesis.
    m_localMultipleBallModel.sensorUpdate(ball.relativePositionOnField,
                                          ball.relativePositionOnField.norm(), // Distance from robot to ball percept
                                          theFrameInfo.time,
                                          ball.validity,
                                          parameters.local.Hypotheses_minDistanceForNewHypothesis,
                                          parameters.local.Hypotheses_initialValidityForNewHypotheses,
                                          kalmanNoiseMatrices);
  }
  else
  {
    // Ball not seen.
    // Ignore invalid ball percept.
  }
}


void BallModelProvider::sensorUpdateRemote()
{
	// Collect ball information from all team mates.
	bool ballSeenByAnyTeamMate = false;
  for (size_t i = 0; i < theTeammateData.teammates.size(); ++i)
  {
    // Loop over all players which have sent data and are active.
    if (theTeammateData.teammates[i].status == Teammate::FULLY_ACTIVE)
    {
      // The ball was recently seen by this teammate.
      bool ballSeenByTeamMate = m_remoteTimeSinceBallSeen[theTeammateData.teammates[i].number] < theTeammateData.teammates[i].ball.timeWhenLastSeen;
      if (ballSeenByTeamMate) ballSeenByAnyTeamMate = true;
      else continue;
      // Save time of last percept which was used to update multiple ball model.
      m_remoteTimeSinceBallSeen[theTeammateData.teammates[i].number] = theTeammateData.teammates[i].ball.timeWhenLastSeen;

      // Get ball position from player i in global coordinates.
      Vector2f percept = Transformation::robotToField(
        theTeammateData.teammates[i].pose,
        theTeammateData.teammates[i].ball.estimate.position);
      Vector2f velocity = Transformation::robotToFieldVelocity(
        theTeammateData.teammates[i].pose,
        theTeammateData.teammates[i].ball.estimate.velocity);
      // Distance from robot to ball percept.
      float distanceToPercept = theTeammateData.teammates[i].ball.lastPerception.norm();

      // Run sensor update with global ball position.
      RemoteKalmanPositionHypothesis::TeammateInfo teammate(
        theTeammateData.teammates[i].ball.validity, theFrameInfo.time);
      m_remoteMultipleBallModel.sensorUpdate(
        percept,
        distanceToPercept,
        &velocity,
        theTeammateData.teammates[i].ball.timeWhenLastSeen,
        theTeammateData.teammates[i].ball.validity,
        parameters.remote.Hypotheses_minDistanceForNewHypothesis,
        parameters.remote.Hypotheses_initialValidityForNewHypotheses,
        kalmanNoiseMatrices,
        theTeammateData.teammates[i].number,
        teammate);
    }
  }
  
  // Update validity of all hypotheses.
  m_remoteMultipleBallModel.updateValidity(
    parameters.remote.Validity_maxPerceptsPerSecond, 
    parameters.remote.goodValidity(),
    parameters.remote.Validity_weightOfPreviousValidity,
    parameters.remote.Validity_weightOfPreviousValidity_goodHypotheses);
}

void BallModelProvider::handleGameState()
{
  // Transition from SET to PLAYING:
  if (m_lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING &&
      parameters.State_SetToPlaying_addKickOffHypothesis) // This can be deactivated via config file.
  {
    // Add a ball hypothesis at the kick off point.
    Vector2f kickOffPoint;
    kickOffPoint << theFieldDimensions.xPosKickOffPoint, theFieldDimensions.yPosKickOffPoint;
    
    // Local ball model:
    // Transform kick off point to relative robot coordinates.
    BallPercept fakePercept;
    fakePercept.relativePositionOnField = Transformation::fieldToRobot(theRobotPose, kickOffPoint);
    fakePercept.validity = theRobotPose.validity;
    for (int i = 0; i < parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis; i++)
      sensorUpdateLocalSingle(fakePercept);
    
    // Remote ball model:
    RemoteKalmanPositionHypothesis newRemoteHypothesis(kalmanNoiseMatrices, 1.f, theFrameInfo.time, 1.f, kickOffPoint);
    // Increase the numberOfSensorUpdates of the new hypothesis to reache the
    // threshold which is required for becoming the best hypothesis.
    newRemoteHypothesis.addNumberOfSensorUpdates(parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis);
    // Add new hypothesis to remote ball model.
    m_remoteMultipleBallModel.addHypothesis(newRemoteHypothesis);
  }
  
  // Penalty is over:
  if (m_lastPenaltyState != PENALTY_NONE
    && m_lastPenaltyState != PENALTY_SPL_ILLEGAL_MOTION_IN_SET
    && theRobotInfo.penalty == PENALTY_NONE &&
      parameters.State_Penalty_removeAllHypotheses) // This can be deactivated via config file.
  {
    m_localMultipleBallModel.clear();
  }
  
  // Save current game state for next iteration.
  m_lastGameState = theGameInfo.state;
  // Save current state of penalty for next iteration.
  m_lastPenaltyState = theRobotInfo.penalty;
}

void BallModelProvider::handleKick()
{
  // TODO: make deactivatable by param
  const float MIN_VELOCITY_FOR_DETECTING_A_KICK = 100.f; // TODO: make param
  
  if (m_localMultipleBallModel.bestHypothesis() != nullptr)
  {
    bool kickTriggered = theMotionRequest.motion == MotionRequest::Motion::kick ||
      (theMotionRequest.motion == MotionRequest::Motion::walk && theMotionRequest.walkRequest.isStepRequestKick());
    if (kickTriggered)
    {
      // If the ball hypothesis already has a high velocity, the kick must be
      // done. Stop changing the covariance; it should decrease now on its own.
      if (m_kickDetected || m_localMultipleBallModel.bestHypothesis()->kalman.velocity().norm() > MIN_VELOCITY_FOR_DETECTING_A_KICK)
        m_kickDetected = true;
      else
      {
        // Increase covarianceMatrix that the max coefficient has a value of (at least) 100.
        // TODO: make param 500.0
        double factor = 500.0 / m_localMultipleBallModel.bestHypothesis()->kalman.covarianceMatrix.block(2,2, 2,2).maxCoeff();
        factor = factor < 1.0 ? 1.0 : factor;
        
        // A new kick was triggered. Increase velocity uncertainty of the best ball
        // hypotheses (the one which should be kicked).
        m_localMultipleBallModel.increaseVelocityUncertainty(factor, true);
      }
    }
    else
    {
      m_kickDetected = false;
    }
    
    DECLARE_PLOT("module:BallModelProvider:localMaxVelocityCovariance");
    DECLARE_PLOT("module:BallModelProvider:kickTriggered");
    
    PLOT("module:BallModelProvider:localMaxVelocityCovariance", m_localMultipleBallModel.bestHypothesis()->kalman.covarianceMatrix.block(2,2, 2,2).maxCoeff());
    PLOT("module:BallModelProvider:kickTriggered", (
      !kickTriggered ? 0 :
      !m_kickDetected ? 50 :
      100));
  }

  // Save current motion type (kick, etc.) for next iteration.
  m_lastMotionType = theMotionRequest.motion;
}


void BallModelProvider::updateTimeWhenBallFirstDisappeared(const KalmanPositionHypothesis* hypothesis, BallModel& ballModel)
{
  // TODO: make parameter
  float DISTANCE_TO_BALL_THRESHOLD = 3000.f; // Max distance to see the ball
  float TIMEOUT_PER_METER = 1000.f;
  int ballDisappearedTimeout = static_cast<int>(TIMEOUT_PER_METER);
  
  // Ball is lost, if it was never seen.
  if (hypothesis == nullptr || hypothesis->timeWhenLastSeen == 0)
  {
    m_ballDisappeared = true;
    m_timeWhenBallFirstDisappeared = 0;
    ballModel.timeWhenDisappeared = 0;
    return;
  }
  
  // Ball is not lost, if it is currently seen.
  if (hypothesis->timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0)
  {
    m_ballDisappeared = false;
    m_timeWhenBallFirstDisappeared = theFrameInfo.time;
    ballModel.timeWhenDisappeared = theFrameInfo.time;
    return;
  }
  
  
  // Should the ball be visible in any of the current camera images?
  bool ballShouldBeVisible =
  Geometry::ballShouldBeVisibleInImage(hypothesis->kalman.position(),
                                       theFieldDimensions.ballRadius,
                                       theCameraMatrix, theCameraInfo) || // lower camera
  Geometry::ballShouldBeVisibleInImage(hypothesis->kalman.position(),
                                       theFieldDimensions.ballRadius,
                                       theCameraMatrixUpper, theCameraInfo); // upper camera
  
  // Get distance to ball and appropriate timeout. Timeout increases with distance.
  float distanceToBallInMeter = hypothesis->kalman.position().norm() / 1000.f;
  // timeout = MIN_THRESHOLD * distance_in_meter
  ballDisappearedTimeout = std::max(static_cast<int>(TIMEOUT_PER_METER * distanceToBallInMeter),
                                    static_cast<int>(TIMEOUT_PER_METER));
  
  
  // If the ball should be visible in the current camera images, but is not
  // AND the distance to the ball is not too far, it is lost.
  if(ballShouldBeVisible &&
     distanceToBallInMeter < DISTANCE_TO_BALL_THRESHOLD / 1000.f)
  {
    m_ballDisappeared = true;
  }
  
  // If the ball is not lost after all, the timeWhenBallFirstDisappeared is set
  // to the current time.
  else if (!m_ballDisappeared)
  {
    m_timeWhenBallFirstDisappeared = theFrameInfo.time;
  }

  // Ball lost since <timeout> milli seconds. Then it is really lost.
  // Set it to the ball model.
  if (theFrameInfo.getTimeSince(m_timeWhenBallFirstDisappeared) > ballDisappearedTimeout)
    ballModel.timeWhenDisappeared = m_timeWhenBallFirstDisappeared;
  else
    ballModel.timeWhenDisappeared = theFrameInfo.time;
}

void BallModelProvider::generateLocalBallModel()
{
  const KalmanPositionHypothesis* bestHypothesis = m_localMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
  {
    updateEstimatedBallState(*bestHypothesis, m_localBallModel.estimate); // set ballModel.estimate
    m_localBallModel.timeWhenLastSeen = bestHypothesis->timeWhenLastSeen;
    m_localBallModel.validity = bestHypothesis->validity;
    m_localBallModel.seenPercentage = static_cast<unsigned char>(
      (static_cast<float>(bestHypothesis->perceptsPerSecond()) / 30.f) * 100.f);
    m_localBallModel.lastPerception = bestHypothesis->lastPerception;
  }
  else
  {
    m_localBallModel.estimate = BallState();
    m_localBallModel.timeWhenLastSeen = 0;
    m_localBallModel.validity = 0.f;
    m_localBallModel.seenPercentage = 0;
    m_localBallModel.lastPerception = Vector2f::Zero();
  }
  updateTimeWhenBallFirstDisappeared(bestHypothesis, m_localBallModel); // set ballModel.timeWhenDisappeared
  
  // Update timeWhenLastSeenByTeamMate from remote ball model.
  bestHypothesis = m_remoteMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
    m_localBallModel.timeWhenLastSeenByTeamMate = bestHypothesis->timeWhenLastSeen;
  else
    m_localBallModel.timeWhenLastSeenByTeamMate = 0;
}

void BallModelProvider::generateRemoteBallModel()
{
  const RemoteKalmanPositionHypothesis* bestHypothesis = m_remoteMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
  {
    BallState bs;
    updateEstimatedBallState(*bestHypothesis, bs);
    m_remoteBallModel.position = bs.position;
    m_remoteBallModel.velocity = bs.velocity;
    m_remoteBallModel.timeWhenLastSeen = bestHypothesis->timeWhenLastSeen;
    m_remoteBallModel.timeWhenLastSeenByGoalie = bestHypothesis->timeWhenUpdatedByTeammate(1);
    m_remoteBallModel.validity = bestHypothesis->validity;
    m_remoteBallModel.teammates = bestHypothesis->teammatesString();
  }
  else
  {
    m_remoteBallModel.position = Vector2f::Zero();
    m_remoteBallModel.velocity = Vector2f::Zero();
    m_remoteBallModel.timeWhenLastSeen = 0;
    m_remoteBallModel.timeWhenLastSeenByGoalie = 0;
    m_remoteBallModel.validity = 0.f;
    m_remoteBallModel.teammates = "";
  }
}
  

// MARK: Debug methods

void BallModelProvider::initModify()
{
  MODIFY("module:BallModelProvider:parameters", parameters);
  MODIFY("module:BallModelProvider:kalmanMatrices", kalmanNoiseMatrices);
  MODIFY("module:BallModelProvider:localHypotheses", m_localMultipleBallModel);
  MODIFY("module:BallModelProvider:remoteHypotheses", m_remoteMultipleBallModel);
}

void BallModelProvider::initDebugDrawing() const
{
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:velocities", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:covariances", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField");
}

void BallModelProvider::draw()
{
  DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField")
  {
    m_localMultipleBallModel.draw();
  }

  DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField")
  {
	  m_remoteMultipleBallModel.draw();
  }

  /*
  // Compare with GroundTruthBallModel for evaluation.
  DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField")
  {
    if (m_localMultipleBallModel.bestHypothesis() != nullptr)
    {
      float deltaVel = m_localMultipleBallModel.bestHypothesis()->kalman.velocity().norm()
        - theGroundTruthBallModel.estimate.velocity.norm();
      DRAWTEXT("module:BallModelProvider:local:hypotheses",
        100, 1000,
        150, ColorRGBA::black, theGroundTruthBallModel.estimate.velocity.norm() << " mm/s  dv=" << deltaVel << " mm/s");
    }
  }
  */
}
