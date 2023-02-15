/**
 * \file BallModelProvider.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 */

#include "BallModelProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/Annotation.h"

MAKE_MODULE(BallModelProvider, modeling)


// MARK: update methods for representations

void BallModelProvider::update(BallModel& ballModel)
{
  execute();
  ballModel = m_localBallModel;
}

void BallModelProvider::update(MultipleBallModel& multipleBallModel)
{
  execute();
  multipleBallModel = m_multipleBallModel;
}

void BallModelProvider::update(RemoteBallModel& remoteBallModel)
{
  execute();
  remoteBallModel = m_remoteBallModel;
}

void BallModelProvider::update(TeamBallModel& teamBallModel)
{
  execute();
  m_teamBallModelProvider.generateTeamBallModel(teamBallModel, m_localMultipleBallModel, m_remoteMultipleBallModel, theRobotPose, theFrameInfo);
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
  m_localMultipleBallModel.setParametersForUpdateBestHypothesis(parameters.local.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
      parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis, // and at least this amount of sensor updates.
      parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);
  m_remoteMultipleBallModel.setParametersForUpdateBestHypothesis(parameters.remote.Hypotheses_minValidityForChangingBestHypothesis, // Changing bestHypothesis needs at least this validity
      parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesisRemote, // and at least this amount of sensor updates.
      parameters.Hypotheses_decreaseValidityOnChangingBestHypothesis);

  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    gyroDataBuffersX[i].fill(theJoinedIMUData.imuData[i].gyro.x());
    gyroDataBuffersY[i].fill(theJoinedIMUData.imuData[i].gyro.y());
  }
}

void BallModelProvider::execute()
{
  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    gyroDataBuffersX[i].push_front(theJoinedIMUData.imuData[i].gyro.x());
    gyroDataBuffersY[i].push_front(theJoinedIMUData.imuData[i].gyro.y());
  }

  Angle gyroVariance = (gyroDataBuffersY[anglesource].getVariance() + gyroDataBuffersX[anglesource].getVariance());
  PLOT("module:BallModelProvider:gyroVariance", gyroVariance.toDegrees());
  PLOT("module:BallModelProvider:gyroThreshold", gyroMaxVariance.toDegrees());
  isStable = gyroVariance < gyroMaxVariance ? true : false;

  // Set RobotPose and CameraMatrix
  m_localMultipleBallModel.setRobotPoseCameraMatrixAndFriction(theRobotPose, theCameraMatrixUpper, ballFriction);
  m_remoteMultipleBallModel.setRobotPoseCameraMatrixAndFriction(theRobotPose, theCameraMatrixUpper, ballFriction);

  // --- Debug methods ---
  DEBUG_RESPONSE_ONCE("module:BallModelProvider:reset")
  {
    m_localMultipleBallModel.clear();
    m_remoteMultipleBallModel.clear();
    perceptBuffer.clear();
  }

  // Allow modification of some parameters in SimRobot.
  initModify();
  // Init debug drawings.
  initDebugDrawing();

  // Only update once per step.
  if (m_lastTimeStamp == theFrameInfo.time)
  {
    return;
  }

  if (theFieldDimensions.xPosOpponentGroundline != 0)
  {
    houghLineDetector.setParameters(anglePrecisionLineDetection,
        distancePrecisionLineDetection,
        minPointsForLineDetection,
        minPointsForLineDetectionWalking,
        static_cast<unsigned int>(theFieldDimensions.xPosOpponentGroundline * 2.f),
        static_cast<unsigned int>(theFieldDimensions.yPosLeftSideline * 2.f));
    ransacLineFitter.setParameters(anglePrecisionLineDetection, distancePrecisionLineDetection, minPointsForLineDetection, minPointsForLineDetectionWalking);
  }

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
    m_localMultipleBallModel.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.CleanUpHypotheses_fieldBorderThreshold);
    m_remoteMultipleBallModel.cleanUpHypothesesOutsideField(theFieldDimensions, theRobotPose, parameters.CleanUpHypotheses_fieldBorderThreshold);
  }

  m_localMultipleBallModel.cleanUpHypothesesLowValidity(parameters.CleanUpHypotheses_belowValidity, true);
  m_localMultipleBallModel.cleanUpHypothesesSimilarToBestOne(parameters.local.Hypotheses_mergeAngleDiff);

  m_remoteMultipleBallModel.cleanUpHypothesesLowValidity(parameters.CleanUpHypotheses_belowValidity, true);
  m_remoteMultipleBallModel.cleanUpHypothesesSimilarToBestOne(parameters.remote.Hypotheses_mergeAngleDiff);

  // Create standard BallModel and RemoteBallModel from multiple ball models.
  generateLocalBallModel();
  generateMultipleBallModel();
  generateRemoteBallModel();

  // --- Debug methods ---

  // Do debug drawings.
  draw();


  // --- Save current state for next iteration ---
  m_lastOdometryData = theOdometryData;
  m_lastTimeStamp = theFrameInfo.time;
}

bool BallModelProvider::isBallInGoalBox(const Vector2f& positionOnField)
{
  return (positionOnField.x() > theFieldDimensions.xPosOpponentGroundline - 100.f || positionOnField.x() < theFieldDimensions.xPosOwnGroundline + 100.f)
      && positionOnField.y() < theFieldDimensions.yPosLeftGoal && positionOnField.y() > theFieldDimensions.yPosRightGoal;
}

void BallModelProvider::motionUpdate()
{
  // Calculate odometry change since last iteration.
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;

  // Apply odometry offset to all local ball hypotheses.
  m_localMultipleBallModel.removeOdometry(odometryOffset);

  // ----- Perform prediction step on all (local and remote) hypotheses  -----
  m_localMultipleBallModel.motionUpdate(theFrameInfo.time);
  m_remoteMultipleBallModel.motionUpdate(theFrameInfo.time);
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
  std::vector<BallPercept> perceptsInThisFrame;

  // Update the perceptBuffer with the latest odometry
  Pose2f odometryOffset = theOdometryData - m_lastOdometryData;
  for (std::vector<BallPercept>& bpv : perceptBuffer)
  {
    for (BallPercept& bp : bpv)
    {
      Vector2f& bpOnField = bp.relativePositionOnField;
      bpOnField -= odometryOffset.translation;
      bpOnField.rotate(-odometryOffset.rotation);
    }
  }

  if (theMultipleBallPercept.balls.size() > 0)
  {
    // Use multiple ball percept.
    for (const BallPercept& ball : theMultipleBallPercept.balls)
    {
      sensorUpdateLocalSingle(ball);
      perceptsInThisFrame.push_back(ball);
    }
  }
  else
  {
    // Multiple ball percept is not filled. Use default ball model.
    sensorUpdateLocalSingle(theBallPercept);
    if (theBallPercept.status == BallPercept::seen && theBallPercept.timestamp > 0)
    {
      perceptsInThisFrame.push_back(theBallPercept);
    }
  }
  perceptBuffer.push_front(perceptsInThisFrame);

  DEBUG_DRAWING("module:BallModelProvider:perceptBuffer", "drawingOnField")
  {
    if (perceptBuffer.size() > 0)
    {
      for (const std::vector<BallPercept>& bpv : perceptBuffer)
      {
        for (const BallPercept& bp : bpv)
        {
          CIRCLE("module:BallModelProvider:perceptBuffer",
              bp.relativePositionOnField.x(),
              bp.relativePositionOnField.y(),
              distancePrecisionLineDetection / 2.f,
              2,
              Drawings::solidPen,
              ColorRGBA::white,
              Drawings::solidBrush,
              ColorRGBA(255, 255, 255, 100));
        }
      }
    }
  }

  STOPWATCH("ransacLineFitter:reset")
  {
    ransacLineFitter.reset();
  }

  bool success = false;
  BallPercept rollingBall;
  Vector2f velocity;
  STOPWATCH("ransacLineFitter:execute")
  {
    if (addModelFromPerceptsTowardsRobot)
    {
      success = ransacLineFitter.execute(perceptBuffer, theCameraMatrixUpper, !isStable, rollingBall, velocity);
    }
  }

  if (success)
  {
    BallPercept bp;
    const KalmanPositionHypothesis* best = m_localMultipleBallModel.bestHypothesis();
    if (best)
    {
      bp.relativePositionOnField = best->kalman.position();
      bp.validity = best->validity;
      sensorUpdateLocalSingle(bp, &velocity);
    }
    //perceptBuffer.clear();
  }

  if (m_localMultipleBallModel.bestHypothesis() != nullptr)
  {
    const KalmanPositionHypothesis* bestHypothesis = m_localMultipleBallModel.bestHypothesis();
    bool xNear = bestHypothesis->kalman.position().x() < 450;
    bool yNear = bestHypothesis->kalman.position().y() < 110 && bestHypothesis->kalman.position().y() > -110;

    bool comingTorwardsUsX = (bestHypothesis->kalman.velocity().x() < 0.f && bestHypothesis->kalman.position().x() > 0.f)
        || (bestHypothesis->kalman.velocity().x() > 0.f && bestHypothesis->kalman.position().x() < 0.f);
    //bool comingTorwardsUsY = (bestHypothesis->kalman.velocity().y() < 0.f && bestHypothesis->kalman.position().y() > 0.f) || (bestHypothesis->kalman.velocity().y() > 0.f && bestHypothesis->kalman.position().y() < 0.f);

    bool velocityHigher = bestHypothesis->kalman.velocity().norm() > 150 && (comingTorwardsUsX);
    if (xNear && yNear && velocityHigher)
    {
      m_localMultipleBallModel.stopBestHypothesis();
      //ANNOTATION("BallModel", "Stopped ball because it collides with the robot.");
    }
  }

  // Update validity of all hypotheses.
  m_localMultipleBallModel.updateValidity(
      parameters.local.Validity_maxPerceptsPerSecond, parameters.local.goodValidity(), parameters.local.Validity_weightOfPreviousValidity, parameters.local.Validity_weightOfPreviousValidity_goodHypotheses);
}

void BallModelProvider::sensorUpdateLocalSingle(const BallPercept& ball, const Vector2f* velocity)
{
  if (isStable && interpolationCounter > 0)
    interpolationCounter--;
  else if (!isStable)
    interpolationCounter = stableInterpolationFrames;

  float factor = 1.f + (walkingMaxMeasurementNoiseFactor - 1.f) * (interpolationCounter / (stableInterpolationFrames + std::numeric_limits<float>::epsilon()));
  m_localMultipleBallModel.updateMaxMeasurementNoise(kalmanNoiseMatrices.maxMeasurementNoise * factor, false);

  // Sensor update (correction of kalman filters) cannot be done for local ball
  // hypotheses, if the ball was not perceived.
  if (ball.status == BallPercept::seen)
  {
    // Run sensor update of each local ball hypothesis.
    m_localMultipleBallModel.sensorUpdate(ball.relativePositionOnField,
        ball.relativePositionOnField.norm(), // Distance from robot to ball percept
        velocity,
        theFrameInfo.time,
        ball.validity,
        parameters.local.Hypotheses_mergeAngleDiff,
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
  for (size_t i = 0; i < theTeammateData.teammates.size(); ++i)
  {
    // Loop over all players which have sent data and are active.
    if (theTeammateData.teammates[i].status == Teammate::FULLY_ACTIVE)
    {
      // The ball was recently seen by this teammate.
      bool ballSeenByTeamMate = m_remoteTimeSinceBallSeen[theTeammateData.teammates[i].number] < theTeammateData.teammates[i].ball.timeWhenLastSeen;
      if (!ballSeenByTeamMate)
        continue;
      // Save time of last percept which was used to update multiple ball model.
      m_remoteTimeSinceBallSeen[theTeammateData.teammates[i].number] = theTeammateData.teammates[i].ball.timeWhenLastSeen;

      // Get ball position from player i in global coordinates.
      Vector2f percept = Transformation::robotToField(theTeammateData.teammates[i].pose, theTeammateData.teammates[i].ball.estimate.position);
      Vector2f velocity = Transformation::robotToFieldVelocity(theTeammateData.teammates[i].pose, theTeammateData.teammates[i].ball.estimate.velocity);
      // Distance from robot to ball percept.
      float distanceToPercept = theTeammateData.teammates[i].ball.estimate.position.norm();
      float robotPoseBasedValidity = theTeammateData.teammates[i].ball.validity * theTeammateData.teammates[i].pose.validity;

      // Run sensor update with global ball position.
      RemoteKalmanPositionHypothesis::TeammateInfo teammate(robotPoseBasedValidity, theFrameInfo.time);
      m_remoteMultipleBallModel.sensorUpdate(percept,
          distanceToPercept,
          &velocity,
          theTeammateData.teammates[i].ball.timeWhenLastSeen,
          robotPoseBasedValidity,
          parameters.remote.Hypotheses_mergeAngleDiff,
          robotPoseBasedValidity, //parameters.remote.Hypotheses_initialValidityForNewHypotheses,
          kalmanNoiseMatrices,
          theTeammateData.teammates[i].number,
          teammate);
    }
  }

  // Update validity of all hypotheses.
  m_remoteMultipleBallModel.updateValidity(
      parameters.remote.Validity_maxPerceptsPerSecond, parameters.remote.goodValidity(), parameters.remote.Validity_weightOfPreviousValidity, parameters.remote.Validity_weightOfPreviousValidity_goodHypotheses);
}

void BallModelProvider::handleGameState()
{
  // Transition from SET to PLAYING:
  if (m_lastGameState == STATE_SET && theGameInfo.state == STATE_PLAYING && parameters.State_SetToPlaying_addKickOffHypothesis) // This can be deactivated via config file.
  {
    // Add a ball hypothesis at the kick off point.
    Vector2f kickOffPoint;
    kickOffPoint << theFieldDimensions.xPosKickOffPoint, theFieldDimensions.yPosKickOffPoint;
    // Adds fake percepts when position for the ball is known
    addFakePercepts(kickOffPoint);
  }

  // Transition from SET_PLAY_NONE to SET_PLAY_CORNER_KICK or SET_PLAY_GOAL_KICK
  if (m_lastSetPlay == SET_PLAY_NONE && (theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_GOAL_KICK) && parameters.State_SetPlay_addSetPlayHypothesis) // This can be deactivated via config file.
  {
    // Kick off point is dependent on kick off team
    bool ownKickOff = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber;
    // Use newest ball model for decision, where ball went out
    int timeSinceRemoteSeen = theFrameInfo.getTimeSince(m_remoteBallModel.timeWhenLastSeen);
    int timeSinceLocalSeen = theFrameInfo.getTimeSince(m_localBallModel.timeWhenLastSeen);
    if (std::min(timeSinceLocalSeen, timeSinceRemoteSeen) <= parameters.State_SetPlay_maxTimeSinceBallSeen)
    {
      Vector2f lastBallPosition = (timeSinceRemoteSeen < timeSinceLocalSeen) ? m_remoteBallModel.position : Transformation::robotToField(theRobotPose, m_localBallModel.estimate.position);
      // Ball is put in on the side it went out, so take last known ball position as base for that
      bool ballWasLeft = lastBallPosition.y() > 0;
      // Add a ball hypothesis at the kick off point.
      Vector2f kickOffPoint;
      if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK)
      {
        kickOffPoint.x() = ownKickOff ? theFieldDimensions.xPosOpponentGroundline : theFieldDimensions.xPosOwnGroundline;
        kickOffPoint.y() = ballWasLeft ? theFieldDimensions.yPosLeftSideline : theFieldDimensions.yPosRightSideline;
      }
      else // goal free kick
      {
        kickOffPoint.x() = ownKickOff ? theFieldDimensions.xPosOwnGoalArea : theFieldDimensions.xPosOpponentGoalArea;
        kickOffPoint.y() = ballWasLeft ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea;
      }
      // Adds fake percepts when position for the ball is known
      addFakePercepts(kickOffPoint);
    }
  }

  // Penalty is over:
  if (m_lastPenaltyState != PENALTY_NONE && m_lastPenaltyState != PENALTY_SPL_ILLEGAL_MOTION_IN_SET && theRobotInfo.penalty == PENALTY_NONE && parameters.State_Penalty_removeAllHypotheses) // This can be deactivated via config file.
  {
    m_localMultipleBallModel.clear();
  }

  // Save current game state for next iteration.
  m_lastGameState = theGameInfo.state;
  // Save current state of penalty for next iteration.
  m_lastPenaltyState = theRobotInfo.penalty;
  // Save current set play for next iteration
  m_lastSetPlay = theGameInfo.setPlay;
}

void BallModelProvider::addFakePercepts(Vector2f pointOnField)
{
  // Local ball model:
  // Transform kick off point to relative robot coordinates.
  BallPercept fakePercept;
  fakePercept.relativePositionOnField = Transformation::fieldToRobot(theRobotPose, pointOnField);
  fakePercept.validity = theRobotPose.validity; //TODO: Why not parameters.local.Hypotheses_initialValidityForNewHypotheses
  fakePercept.status = BallPercept::seen;
  fakePercept.timestamp = theFrameInfo.time;
  for (int i = 0; i < parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis; i++)
    sensorUpdateLocalSingle(fakePercept);

  // Remote ball model:
  RemoteKalmanPositionHypothesis newRemoteHypothesis(kalmanNoiseMatrices, 1.f, theFrameInfo.time, 1.f, pointOnField, m_remoteMultipleBallModel.perceptDuration);
  // Increase the numberOfSensorUpdates of the new hypothesis to reache the
  // threshold which is required for becoming the best hypothesis.
  newRemoteHypothesis.addNumberOfSensorUpdates(parameters.Hypotheses_minNumberOfSensorUpdatesForBestHypothesis);
  // Add new hypothesis to remote ball model.
  m_remoteMultipleBallModel.addHypothesis(newRemoteHypothesis);
}

void BallModelProvider::handleKick()
{
  // TODO: make deactivatable by param
  const float MIN_VELOCITY_FOR_DETECTING_A_KICK = 100.f; // TODO: make param

  if (m_localMultipleBallModel.bestHypothesis() != nullptr)
  {
    // Use robot model's foot positions intersecting with best hypothesis
    const Pose3f& kickFoot = (theRobotModel.soleLeft.translation.z() > theRobotModel.soleRight.translation.z()) ? theRobotModel.soleLeft : theRobotModel.soleRight;
    Vector2f ballPosition = m_localMultipleBallModel.bestHypothesis()->kalman.position();
    bool footIntersectsWithModel = (ballPosition.y() - kickFoot.translation.y()) < theFieldDimensions.ballRadius && ballPosition.x() > 90.f
        && (ballPosition.x() - kickFoot.translation.x()) < (180.f + theFieldDimensions.ballRadius); // 104 mm is foot length from base

    // Two ways to trigger a kick are handled here: kick engine or in walk kick.
    bool kickTriggered = (theMotionInfo.motion == MotionRequest::Motion::kick || (theMotionInfo.motion == MotionRequest::Motion::walk && theMotionInfo.walkKicking)) && footIntersectsWithModel;
    if (kickTriggered)
    {
      // If the ball hypothesis already has a high velocity, the kick must be
      // done. Stop changing the covariance; it should decrease now on its own.
      if (m_kickDetected || m_localMultipleBallModel.bestHypothesis()->kalman.velocity().norm() > MIN_VELOCITY_FOR_DETECTING_A_KICK)
      {
        m_kickDetected = true;
      }
      else
      {
        /* Works, but should not be necessary at the moment. Better: Replace this with kick detection, i.e. through several percepts on a line.
        // Move the currently best hypothesis in the kick direction. This helps the behavior go after the ball immediately
        // and should provide a faster fit if the ball was hit, as should be expected.
        Vector2f kickTargetRel = Transformation::fieldToRobot(theRobotPose, theMotionInfo.kickRequest.kickTarget);
        Pose2f kickPose(kickTargetRel.angle(),
          m_localMultipleBallModel.bestHypothesis()->kalman.position());
        // TODO: move depending on kick?
        kickPose.translate(300.0, 0);

        m_localMultipleBallModel.clear();
        KalmanPositionHypothesis kickedBallHypothesis(kalmanNoiseMatrices,1.f,theFrameInfo.time,1.f,kickPose.translation,
          m_localMultipleBallModel.getPerceptDuration());
        m_localMultipleBallModel.addHypothesis(kickedBallHypothesis);
        */
        // Increase covarianceMatrix that the max coefficient has a value of (at least) 100.
        // TODO: make param 500.0
        double factor = 10000.0 / m_localMultipleBallModel.bestHypothesis()->kalman.covarianceMatrix.block(2, 2, 2, 2).maxCoeff();

        // A new kick was triggered. Increase velocity uncertainty of the best ball
        // hypotheses (the one which should be kicked).
        m_localMultipleBallModel.increaseUncertainty(1.f, factor, true);
        m_localMultipleBallModel.hypothesisKicked(theMotionInfo.motion == MotionRequest::Motion::kick ? true : false);
      }
    }
    else
    {
      m_kickDetected = false;
    }

    PLOT("module:BallModelProvider:localMaxVelocityCovariance", m_localMultipleBallModel.bestHypothesis()->kalman.covarianceMatrix.block(2, 2, 2, 2).maxCoeff());
    PLOT("module:BallModelProvider:kickTriggered", (!kickTriggered ? 0 : !m_kickDetected ? 50 : 100));
  }

  // Save current motion type (kick, etc.) for next iteration.
  m_lastMotionType = theMotionInfo.motion;
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
  bool ballShouldBeVisible = Geometry::ballShouldBeVisibleInImage(hypothesis->kalman.position(), theFieldDimensions.ballRadius, theCameraMatrix, theCameraInfo) || // lower camera
      Geometry::ballShouldBeVisibleInImage(hypothesis->kalman.position(), theFieldDimensions.ballRadius, theCameraMatrixUpper, theCameraInfo); // upper camera

  // Get distance to ball and appropriate timeout. Timeout increases with distance.
  float distanceToBallInMeter = hypothesis->kalman.position().norm() / 1000.f;
  // timeout = MIN_THRESHOLD * distance_in_meter
  ballDisappearedTimeout = std::max(static_cast<int>(TIMEOUT_PER_METER * distanceToBallInMeter), static_cast<int>(TIMEOUT_PER_METER));


  // If the ball should be visible in the current camera images, but is not
  // AND the distance to the ball is not too far, it is lost.
  if (ballShouldBeVisible && distanceToBallInMeter < DISTANCE_TO_BALL_THRESHOLD / 1000.f)
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
  m_localMultipleBallModel.resetBestHypothesisIndex();
  const KalmanPositionHypothesis* bestHypothesis = m_localMultipleBallModel.bestHypothesis();

  if (bestHypothesis != nullptr)
  {
    updateEstimatedBallState(*bestHypothesis, m_localBallModel.estimate); // set ballModel.estimate
    if (!(m_localBallModel.estimate.position == m_localBallModel.estimate.position))
    {
      m_localBallModel.estimate.position = bestHypothesis->lastPerception;
      m_localBallModel.estimate.velocity = Vector2f::Zero();
    }; //TODO should not happen but I don't know why it sometimes happens..

    m_localBallModel.timeWhenLastSeen = bestHypothesis->timeWhenLastSeen;
    m_localBallModel.validity = bestHypothesis->validity;
    m_localBallModel.seenPercentage = static_cast<unsigned char>((static_cast<float>(bestHypothesis->perceptsPerSecond()) / 30.f) * 100.f);
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
  m_localBallModel.friction = ballFriction;
  updateTimeWhenBallFirstDisappeared(bestHypothesis, m_localBallModel); // set ballModel.timeWhenDisappeared
  if (isBallInGoalBox(Transformation::robotToField(theRobotPose, m_localBallModel.estimate.position)))
    m_localBallModel.timeWhenBallInGoalBox = theFrameInfo.time;

  // Update timeWhenLastSeenByTeamMate from remote ball model.
  bestHypothesis = m_remoteMultipleBallModel.bestHypothesis();
  if (bestHypothesis != nullptr)
    m_localBallModel.timeWhenLastSeenByTeamMate = bestHypothesis->timeWhenLastSeen;
  else
    m_localBallModel.timeWhenLastSeenByTeamMate = 0;
}

void BallModelProvider::generateMultipleBallModel()
{
  m_multipleBallModel.ballModels.clear();
  for (KalmanPositionHypothesis& kph : m_localMultipleBallModel.m_hypotheses)
  {
    float minValidity = parameters.local.goodValidity();
    if (kph.validity > minValidity)
    {
      BallModel bm;
      updateEstimatedBallState(kph, bm.estimate); // set ballModel.estimate
      bm.timeWhenLastSeen = kph.timeWhenLastSeen;
      bm.validity = kph.validity;
      bm.seenPercentage = static_cast<unsigned char>((static_cast<float>(kph.perceptsPerSecond()) / 30.f) * 100.f);
      bm.lastPerception = kph.lastPerception;
      bm.friction = ballFriction;
      bm.timeWhenLastSeenByTeamMate = 0;
      updateTimeWhenBallFirstDisappeared(&kph, bm); // set ballModel.timeWhenDisappeared
      m_multipleBallModel.ballModels.push_back(bm);
    }
  }

  auto sorting_criteria = [](const auto& a, const auto& b)
  {
    if (a.estimate.position.norm() == b.estimate.position.norm())
      return a.validity < b.validity;
    return a.estimate.position.norm() < b.estimate.position.norm();
  };

  std::sort(m_multipleBallModel.ballModels.begin(), m_multipleBallModel.ballModels.end(), sorting_criteria);
}

void BallModelProvider::generateRemoteBallModel()
{
  m_remoteMultipleBallModel.resetBestHypothesisIndex();
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

  if (isBallInGoalBox(m_remoteBallModel.position))
    m_remoteBallModel.timeWhenBallInGoalBox = theFrameInfo.time;
}

// MARK: Debug methods

void BallModelProvider::initModify()
{
  MODIFY("module:BallModelProvider:parameters", parameters);
  MODIFY("module:BallModelProvider:kalmanMatrices", kalmanNoiseMatrices);
  MODIFY("module:BallModelProvider:localHypotheses", m_localMultipleBallModel);
  MODIFY("module:BallModelProvider:remoteHypotheses", m_remoteMultipleBallModel);
  MODIFY("module:BallModelProvider:walkingMaxMeasurementNoiseFactor", walkingMaxMeasurementNoiseFactor);
}

void BallModelProvider::initDebugDrawing() const
{
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:hypotheses", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:velocities", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:local:covariances", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:BallModelProvider:remote:hypotheses", "drawingOnField");

  DECLARE_DEBUG_DRAWING("module:HoughLineDetector:houghLine", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:RANSACLineFitter:line", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:RANSACLineFitter:perceptBuffer", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallModelProvider:perceptBuffer", "drawingOnField");

  DECLARE_PLOT("module:BallModelProvider:localMaxVelocityCovariance");
  DECLARE_PLOT("module:BallModelProvider:kickTriggered");
}

void BallModelProvider::draw()
{
  m_localMultipleBallModel.draw();

  m_remoteMultipleBallModel.draw();

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
