/**
 * \file BallModelProvider.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 */

#include "BallModelProvider.h"

MAKE_MODULE(BallModelProvider, modeling)


// ---------- update methods for representations ----------

void BallModelProvider::update(BallModel& ballModel)
{
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:test", "drawingOnField");
  float validityLoss = 0.01f;
  float validityGain = 0.1f;
  int timeSteps = 90;
  int perceptsPerSecond = 6;
  float startValidity = 0.5f;
  MODIFY("module:BallModelProvider:validityLoss", validityLoss);
  MODIFY("module:BallModelProvider:validityGain", validityGain);
  MODIFY("module:BallModelProvider:timeSteps", timeSteps);
  MODIFY("module:BallModelProvider:perceptsPerSecond", perceptsPerSecond);
  MODIFY("module:BallModelProvider:startValidity", startValidity);
  float val = startValidity;
  for (int i = 0; i < timeSteps; i++)
  {
    if (i%perceptsPerSecond == 0)
      val += validityGain*std::max(0.3f,(1.f - val));
    val = std::min(1.f, val);
    val *= (1-validityLoss);
  }
  DRAWTEXT("module:BallModelProvider:test", 0, 0, 500, ColorRGBA::red, "" << val);
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


// ---------- BallModelProvider methods ----------

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
}

void BallModelProvider::execute()
{
  // Only update once per step.
  if (m_lastTimeStamp == theFrameInfo.time) {
    return;
  }
  // Calculate time since last iteration.
  float timeOffset = static_cast<float>(theFrameInfo.time - m_lastTimeStamp) * 0.001f; // in seconds


  // --- Debug methods ---

  // Allow modification of some parameters in SimRobot.
  initModify();
  // Init debug drawings.
  initDebugDrawing();


  // --- Filter ---
  
  // Add a ball hypothesis to the field center at the transition from game state set to playing.
  handleGameState();
  
  // Perform prediction step.
  motionUpdate(timeOffset);
  // Perform correction step.
  sensorUpdate(timeOffset);

  
  // Update the index of the best ball hypothesis of local and remote ball models.
  m_localMultipleBallModel.updateBestHypothesis(
    parameters.local.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
    parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis,  // and at least this amount of sensor updates.
    parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);
  m_remoteMultipleBallModel.updateBestHypothesis(
    parameters.remote.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
    parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis,   // and at least this amount of sensor updates.
    parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);

  // Remove hypotheses with too small validities and those which are too similar 
  // to the best hypothesis.
  m_localMultipleBallModel.cleanUpHypotheses(
    theFieldDimensions, theRobotPose,
    parameters.Hypotheses_minValidityForKeepingHypothesis,
    parameters.local.Hypotheses_minDistanceForNewHypothesis,
    parameters.Hypotheses_minAngleForKeepingHypothesis);
  m_remoteMultipleBallModel.cleanUpHypotheses(
    theFieldDimensions, theRobotPose,
    parameters.Hypotheses_minValidityForKeepingHypothesis,
    parameters.remote.Hypotheses_minDistanceForNewHypothesis,
    parameters.Hypotheses_minAngleForKeepingHypothesis);
  
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

void BallModelProvider::motionUpdate(float timeOffset)
{
  // Calculate odometry change since last iteration.
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;

  // Apply odometry offset to all local ball hypotheses.
  m_localMultipleBallModel.removeOdometry(odometryOffset);

  // ----- Perform prediction step on all (local and remote) hypotheses  -----
  m_localMultipleBallModel.motionUpdate(timeOffset, theFieldDimensions.ballFriction); // ballFriction < 0
  m_remoteMultipleBallModel.motionUpdate(timeOffset, theFieldDimensions.ballFriction); // ballFriction < 0
}

void BallModelProvider::sensorUpdate(float timeOffset)
{
  // ----- Perform correction step on all local hypotheses -----
  sensorUpdateLocal(timeOffset);

  // ----- Perform correction step on all remote hypotheses -----
  sensorUpdateRemote(timeOffset);
}

void BallModelProvider::sensorUpdateLocal(float timeOffset)
{
  // Add new frame to each hypothesis for computation of percepts/s.
  m_localMultipleBallModel.initializeFrameForValidity();

	// Sensor update (correction of kalman filters) cannot be done for local ball 
	// hypotheses, if the ball was not perceived. 
	if (theBallPercept.status == BallPercept::seen)
	{
		// Save percept in both ball models as lastPerception if ball was seen.
		m_localBallModel.lastPerception = theBallPercept.relativePositionOnField;

		// Run sensor update of each local ball hypothesis.
		m_localMultipleBallModel.sensorUpdate(
			theBallPercept.relativePositionOnField,
			theBallPercept.relativePositionOnField.norm(), // Distance from robot to ball percept
			theFrameInfo.time,
			1.f, // theBallPercept.validity is not filled actually
			parameters.local.Hypotheses_minDistanceForNewHypothesis,
			parameters.Hypotheses_initialValidityForNewHypotheses,
      kalmanMatrices);
	}
  else
  {
    // Ball not seen. TODO?
  }
  
  // Update validity of all hypotheses.
  m_localMultipleBallModel.updateValidity(
    parameters.local.Validity_maxPerceptsPerSecond,
    parameters.local.goodValidity(),
    parameters.local.Validity_weightOfPreviousValidity,
    parameters.local.Validity_weightOfPreviousValidity_goodHypotheses);
}

void BallModelProvider::sensorUpdateRemote(float timeOffset)
{
  // Add new frame to each hypothesis for computation of percepts/s.
  m_remoteMultipleBallModel.initializeFrameForValidity();

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
      BallHypothesis::BallHypothesisTeammateInfo teammate(
        theTeammateData.teammates[i].ball.validity, theFrameInfo.time);
      m_remoteMultipleBallModel.sensorUpdate(
        percept,
        distanceToPercept,
        &velocity,
        theTeammateData.teammates[i].ball.timeWhenLastSeen,
        1.f,
        parameters.remote.Hypotheses_minDistanceForNewHypothesis,
        parameters.Hypotheses_initialValidityForNewHypotheses,
        kalmanMatrices,
        theTeammateData.teammates[i].number,
        &teammate);
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
    Vector2f relativeKickOffPoint = Transformation::fieldToRobot(theRobotPose, kickOffPoint);
    BallHypothesis newLocalHypothesis(relativeKickOffPoint, theRobotPose.validity, theFrameInfo.time, kalmanMatrices);
    // Increase the numberOfSensorUpdates of the new hypothesis to reache the
    // threshold which is required for becoming the best hypothesis.
    newLocalHypothesis.addNumberOfSensorUpdates(parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis);
    // Add new hypothesis to local ball model.
    m_localMultipleBallModel.addHypothesis(newLocalHypothesis);
    
    // Remote ball model:
    BallHypothesis newRemoteHypothesis(kickOffPoint, 1.f, theFrameInfo.time, kalmanMatrices);
    // Increase the numberOfSensorUpdates of the new hypothesis to reache the
    // threshold which is required for becoming the best hypothesis.
    newLocalHypothesis.addNumberOfSensorUpdates(parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis);
    // Add new hypothesis to remote ball model.
    m_remoteMultipleBallModel.addHypothesis(newRemoteHypothesis);
  }
  
  // Penalty is over:
  if (m_lastPenaltyState != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE &&
      parameters.State_Penalty_removeAllHypotheses) // This can be deactivated via config file.
  {
    m_localMultipleBallModel.clear();
    m_remoteMultipleBallModel.clear();
  }
  
  
  // Save current game state for next iteration.
  m_lastGameState = theGameInfo.state;
  // Save current state of penalty for next iteration.
  m_lastPenaltyState = theRobotInfo.penalty;
}

void BallModelProvider::updateTimeWhenBallFirstDisappeared(const BallHypothesis* hypothesis, BallModel& ballModel)
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
  const BallHypothesis* bestHypothesis = m_localMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
  {
    bestHypothesis->updateEstimatedBallState(m_localBallModel.estimate); // set ballModel.estimate
    m_localBallModel.timeWhenLastSeen = bestHypothesis->timeWhenLastSeen;
    m_localBallModel.validity = bestHypothesis->validity;
    m_localBallModel.seenPercentage = static_cast<unsigned char>((static_cast<float>(bestHypothesis->perceptsPerSecond()) / 30.f) * 100.f);
  }
  else
  {
    m_localBallModel.estimate = BallState();
    m_localBallModel.timeWhenLastSeen = 0;
    m_localBallModel.validity = 0.f;
    m_localBallModel.seenPercentage = 0;
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
  const BallHypothesis* bestHypothesis = m_remoteMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
  {
    BallState bs;
    bestHypothesis->updateEstimatedBallState(bs);
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
  

// ---------- Debug methods ----------

void BallModelProvider::initModify()
{
  MODIFY("module:BallModelProvider:parameters", parameters);
  MODIFY("module:BallModelProvider:kalmanMatrices", kalmanMatrices);
  MODIFY("module:BallModelProvider:localHypotheses", m_localMultipleBallModel);
  MODIFY("module:BallModelProvider:remoteHypotheses", m_remoteMultipleBallModel);
}

void BallModelProvider::initDebugDrawing() const
{
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:velocities", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField");
}

void BallModelProvider::draw() const
{
  DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField")
  {
    m_localMultipleBallModel.draw();
  }

  DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField")
  {
	  m_remoteMultipleBallModel.draw();
  }

  // Compare with GroundTruthBallModel for evaluation.
  /*
  DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField")
  {
    float deltaVel = m_localMultipleBallModel.bestHypothesis()->kalman.velocity().abs()
      - theGroundTruthBallModel.estimate.velocity.abs();
    DRAWTEXT("module:BallModelProvider:local:hypotheses",
      100, 1000,
      150, ColorClasses::black, theGroundTruthBallModel.estimate.velocity.abs() << " mm/s  dv=" << deltaVel << " mm/s");
  }
  */
}
