#include "CognitionMindfulness.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <random>
#include <float.h>


MAKE_MODULE(CognitionMindfulness, perception)

CognitionMindfulness::CognitionMindfulness()
{
  ballStartPosition = theBallSymbols.ballPositionField;
  ballEndPosition = theBallSymbols.ballPositionField;
  kickTimeStart = 0;
  kickTimeStop = 0;
  highestKickPathDist = 0.0;

  firstPercentage = theSystemSensorData.batteryLevel;
  lastPercentage = firstPercentage;
  startTimestamp = theFrameInfo.time;
}

void CognitionMindfulness::update(CognitionState& theCognitionState)
{
  if (!activate)
    return;

  checkForLocaProblem(theCognitionState);
  checkForPenaltyProblem(theCognitionState);
  checkForBallLostAfterKick(theCognitionState);

  checkImageBrightness(theCognitionState);

  kickDistanceDetection(theCognitionState);
  kickPathAndTimeDetection(theCognitionState);
  STOPWATCH("module:CognitionMindfulness:kickDistanceSimulation")
  kickDistanceSimulation(theCognitionState);
}

bool CognitionMindfulness::hasProblem(CognitionState& cognitionState, CognitionState::CognitionStateError cognitionStateError)
{
  for (auto& problem : cognitionState.cognitionProblems)
    if (problem == cognitionStateError)
      return true;
  return false;
}

void CognitionMindfulness::checkForLocaProblem(CognitionState& cognitionState)
{
  /***********************************/
  /* reset values                    */
  /***********************************/
  int currentTime = theFrameInfo.time;
  // 30 seconds or more till last penalty
  if (std::abs(currentTime - lastLocaPenalty) >= 30000)
  {
    auto it = std::find(cognitionState.cognitionProblems.begin(), cognitionState.cognitionProblems.end(), CognitionState::locaProblem);
    if (it != cognitionState.cognitionProblems.end())
      cognitionState.cognitionProblems.erase(it);
    locaScore = 0;
  }

  // TODO need feedback from behavior when the localization should work again
  // to start the 30 seconds if the localization should work
  if (theRobotInfo.penalty == PENALTY_NONE)
    locaPenaltyActive = false;
  else
    lastLocaPenalty = theFrameInfo.time;

  /***********************************/
  /* update scoring                  */
  /***********************************/
  if (!locaPenaltyActive && theRobotInfo.penalty == PENALTY_SPL_LEAVING_THE_FIELD)
  {
    if (locaScore < locaScoreLimit)
      locaScore += 2.f;
    locaPenaltyActive = true;
    lastLocaPenalty = theFrameInfo.time;
  }
  if (!locaPenaltyActive && theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_POSITION_IN_SET)
  {
    if (locaScore < locaScoreLimit)
      locaScore += 1.f;
    locaPenaltyActive = true;
    lastLocaPenalty = theFrameInfo.time;
  }
  // Maybe useless because of changed rules
  if (!locaPenaltyActive && theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_POSITION)
  {
    if (locaScore < locaScoreLimit)
      locaScore += 0.5f;
    locaPenaltyActive = true;
    lastLocaPenalty = theFrameInfo.time;
  }

  if (locaScore >= locaScoreLimit)
    if (!hasProblem(cognitionState, CognitionState::locaProblem))
      cognitionState.cognitionProblems.push_back(CognitionState::locaProblem);

  /***********************************/
  /* update status                   */
  /***********************************/
  cognitionState.locaStatus.locaScore = locaScore;
}

void CognitionMindfulness::checkForPenaltyProblem(CognitionState& cognitionState)
{
  if (theRobotInfo.penalty == PENALTY_NONE)
    penaltyActive = false;

  /***********************************/
  /* update scoring                  */
  /***********************************/
  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_BALL_CONTACT)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }

  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }

  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_LOCAL_GAME_STUCK)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }

  // Distance to other Robots to short?
  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_PLAYER_PUSHING)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }

  // Robot broken or other issues
  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_REQUEST_FOR_PICKUP)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }
  if (!penaltyActive && theRobotInfo.penalty == PENALTY_SPL_INACTIVE_PLAYER)
  {
    penaltyScore += 1;
    penaltyActive = true;
  }

  /***********************************/
  /* update status                   */
  /***********************************/
  cognitionState.penaltyStatus.penaltyScore = penaltyScore;
}

void CognitionMindfulness::checkForBallLostAfterKick(CognitionState& cognitionState)
{
  if (theMotionRequest.motion == MotionRequest::kick || theMotionInfo.walkKicking)
    currentKickTime = theFrameInfo.time;

  // Check if Ball is lost in a timewindow of 2 seconds after kick plus time to kick
  if ((currentKickTime + 2000) > theFrameInfo.time)
  {
    if (!theBallSymbols.ballWasSeen)
      ballLost = true;
    else
      ballLost = false;
    ballLostChange = true;
  }
  else
  {
    if (ballLost && ballLostChange)
    {
      ballLostCounter += 1;
      ballLostChange = false;
    }
  }

  /***********************************/
  /* update status                   */
  /***********************************/
  cognitionState.ballStatus.ballLostAfterKick = ballLostCounter;
}

void CognitionMindfulness::checkImageBrightness(CognitionState& cognitionState)
{
  /***********************************/
  /* reset failure                   */
  /***********************************/
  for (int problem = CognitionState::lowerCameraBrightnessLow; problem <= CognitionState::upperCameraBrightnessBright; problem++)
  {
    auto it = std::find(cognitionState.cognitionProblems.begin(), cognitionState.cognitionProblems.end(), problem);
    if (it != cognitionState.cognitionProblems.end())
      cognitionState.cognitionProblems.erase(it);
  }

  /***********************************/
  /* get field color brightness      */
  /***********************************/
  int lowerFieldColorOptY = theFieldColors.fieldColorArray[0].fieldColorOptY;
  int upperFieldColorOptY = theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;

  //TODO Maybe it is better to count bad frames and then throw an problem
  if (lowerFieldColorOptY < lowerBrightnessLowThreshold)
    if (!hasProblem(cognitionState, CognitionState::lowerCameraBrightnessLow))
      cognitionState.cognitionProblems.push_back(CognitionState::lowerCameraBrightnessLow);

  if (lowerFieldColorOptY > lowerBrightnessBrightThreshold)
    if (!hasProblem(cognitionState, CognitionState::lowerCameraBrightnessBright))
      cognitionState.cognitionProblems.push_back(CognitionState::lowerCameraBrightnessBright);

  if (upperFieldColorOptY < upperBrightnessLowThreshold)
    if (!hasProblem(cognitionState, CognitionState::upperCameraBrightnessLow))
      cognitionState.cognitionProblems.push_back(CognitionState::upperCameraBrightnessLow);

  if (upperFieldColorOptY > upperBrightnessBrightThreshold)
    if (!hasProblem(cognitionState, CognitionState::upperCameraBrightnessBright))
      cognitionState.cognitionProblems.push_back(CognitionState::upperCameraBrightnessBright);

  /***********************************/
  /* update status                   */
  /***********************************/
  cognitionState.imageStatus.lowerCameraBrightness = lowerFieldColorOptY;
  cognitionState.imageStatus.upperCameraBrightness = upperFieldColorOptY;
}

void CognitionMindfulness::checkBattery(CognitionState& cognitionState)
{
  if (theSystemSensorData.chargingStatus)
  {
    firstPercentage = theSystemSensorData.batteryLevel;
    startTimestamp = theFrameInfo.time;
  }

  lastPercentage = theSystemSensorData.batteryLevel;
  // Assumption one minute without charging should drain the battery of at least one percent
  if (!theSystemSensorData.chargingStatus && theFrameInfo.getTimeSince(startTimestamp) > 60000)
    cognitionState.batteryStatus.batteryBroken = firstPercentage == lastPercentage;
}

void CognitionMindfulness::kickDistanceDetection(CognitionState& cognitionState)
{
  /***********************************/
  /* get kick IDs                    */
  /***********************************/
  if (theMotionRequest.motion == MotionRequest::kick)
  {
    cognitionState.ballStatus.isKickEngine = true;
    cognitionState.ballStatus.kickEngineID = theMotionRequest.kickRequest.kickMotionType;
  }
  /*if (theMotionInfo.customStepKickInPreview)
  {
    cognitionState.ballStatus.isKickEngine = false;
    cognitionState.ballStatus.walkKickID = theMotionRequest.walkRequest.stepRequest;
  }*/

  /***********************************/
  /* estimate distance               */
  /***********************************/
  if (theMotionRequest.motion == MotionRequest::kick) // || theMotionInfo.walkKicking)
  {
    ballStartPosition = theBallSymbols.ballPositionField;
    ballEndPosition = ballStartPosition;
    lastBallEndPosition = ballEndPosition;
    kicked = true;
    timeKicked = theFrameInfo.time;
    kickDistance = 0;
  }
  else
  {
    ballEndPosition = theBallSymbols.ballPositionField;
    if (kicked)
    {
      // If the ball has been kicked and is moving
      // and the kick was not more than 5 seconds ago.
      // Otherwise the shot will be marked as completed after 10 Frames.
      if ((ballEndPosition - lastBallEndPosition).norm() > 50 && (timeKicked + 5000) > theFrameInfo.time)
      {
        lastBallEndPosition = ballEndPosition;
        kickedCount = 0;
      }
      else
      {
        kickedCount++;
      }

      if (kickedCount > 30)
      {
        kickDistance = (ballEndPosition - ballStartPosition).norm();
        kicked = false;
        kickedCount = 0;
      }
    }
  }

  /***********************************/
  /* update status                   */
  /***********************************/
  if (!kicked)
  {
    cognitionState.ballStatus.estimatedKickDistance = kickDistance;
    rollResistanceEstimation(cognitionState);
  }
}

void CognitionMindfulness::kickPathAndTimeDetection(CognitionState& cognitionState)
{
  Pose3f kickFoot;
  if (theRobotModel.soleLeft.translation.z() > theRobotModel.soleRight.translation.z())
    kickFoot = theRobotModel.soleLeft;
  else
    kickFoot = theRobotModel.soleRight;

  if (firstPos)
  {
    sinkPos = kickFoot.translation;
    sinkTime = theFrameInfo.time;
    firstPos = false;
  }

  sourcePos = kickFoot.translation;
  sourceTime = theFrameInfo.time;
  Vector3f distVec = sinkPos - sourcePos;
  unsigned bufferdTime = sourceTime - sinkTime;
  sinkPos = sourcePos;
  sinkTime = sourceTime;

  Vector2f ballPos = theBallModel.estimate.position;
  bool footIntersectsWithBall = (ballPos.y() - kickFoot.translation.y()) < theFieldDimensions.ballRadius && ballPos.x() > 80.f
      && (ballPos.x() - kickFoot.translation.x()) < (100.f + theFieldDimensions.ballRadius);

  if (cooldown > 1000)
  {
    distVecXFilter.push_front(distVec.x());
    distVecNormFilter.push_front(distVec.norm());

    distVecXBuffer.push_front(distVecXFilter.average());
    distVecNormBuffer.push_front(distVecNormFilter.average());
    distVecTimeBuffer.push_front(bufferdTime);

    if (theMotionInfo.motion == theMotionInfo.kick) // || theMotionInfo.walkKicking
      kickIsStarted = true;

    if (prevFootIntersectsWithBall && !footIntersectsWithBall && kickIsStarted)
    {
      double distance = 0;
      unsigned time = 0;
      bool firstBackFinished = false;
      for (size_t i = 0; i < distVecXBuffer.size(); i++)
      {
        if (!firstBackFinished && distVecXBuffer[i] >= 0)
          continue;
        else if (!firstBackFinished && distVecXBuffer[i] < 0)
          firstBackFinished = true;

        if (distVecXBuffer[i] >= 0)
          break;

        time += distVecTimeBuffer[i];
        distance += distVecNormBuffer[i];
      }

      cognitionState.ballStatus.swingTime = time;
      cognitionState.ballStatus.swingDistance = distance;
      kickDataUpdated = true;
      cooldown = 0;
      kickIsStarted = false;
    }
  }

  cooldown += bufferdTime;
  prevFootIntersectsWithBall = footIntersectsWithBall;
}

void CognitionMindfulness::rollResistanceEstimation(CognitionState& cognitionState)
{
  if (cognitionState.ballStatus.estimatedKickDistance != 0.0 && cognitionState.ballStatus.estimatedKickDistance != oldEstimatedKickDistance)
  {
    lastDistances.push_front(cognitionState.ballStatus.estimatedKickDistance);
    lastKickTimes.push_front(cognitionState.ballStatus.swingTime);
    lastKickPathDistances.push_front(cognitionState.ballStatus.swingDistance);
    oldEstimatedKickDistance = cognitionState.ballStatus.estimatedKickDistance;
  }

  if (lastDistances.full() && lastKickTimes.full() && lastKickPathDistances.full())
  {
    Vector2f source(0, 0);
    Vector2f target(0, 1);

    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> randomValue{0.0, 0.1};
    std::normal_distribution<> stepValue{0.0, 0.1};

    //double lastRollResistance = upperRollResistance + 1.0;
    int count = 0;
    double sigma = 1.0;
    while (count < 10)
    {
      std::vector<double> rollResitanceCandidates;
      std::vector<double> sigmaCandidates;
      rollResitanceCandidates.push_back(rollResistance);
      sigmaCandidates.push_back(sigma);
      for (size_t i = 0; i < 10; i++)
      {
        double rollResistanceCandidate = rollResistance;
        sigma *= std::exp(stepValue(gen));
        rollResistanceCandidate += sigma * randomValue(gen);
        if (rollResistanceCandidate < lowerRollResistance)
          rollResistanceCandidate = lowerRollResistance;
        if (rollResistanceCandidate > upperRollResistance)
          rollResistanceCandidate = upperRollResistance;
        rollResitanceCandidates.push_back(rollResistanceCandidate);
        sigmaCandidates.push_back(sigma);
      }

      size_t bestIndex = 0;
      double lastBallDistanceDifferenceSum = DBL_MAX;
      for (size_t i = 0; i < rollResitanceCandidates.size(); i++)
      {
        rollResistance = rollResitanceCandidates[i];
        double ballDistanceDifferenceSum = 0.0;
        for (size_t j = 0; j < lastDistances.size(); j++)
        {
          Vector2f ballPositionPrediction = kickSimulation(1, target, source, lastKickPathDistances[j], lastKickTimes[j], true)[0];
          ballDistanceDifferenceSum += std::abs(ballPositionPrediction.norm() - lastDistances[j]);
        }

        if (ballDistanceDifferenceSum < lastBallDistanceDifferenceSum)
        {
          bestIndex = i;
          lastBallDistanceDifferenceSum = ballDistanceDifferenceSum;
        }
      }

      rollResistance = rollResitanceCandidates[bestIndex];
      sigma = sigmaCandidates[bestIndex];
      count++;
    }
  }

  if (rollResistance > prevRollResitance + 0.01 || rollResistance < prevRollResitance - 0.01)
  {
    prevRollResitance = rollResistance;
    rollResistanceBuffer.push_front(rollResistance);

    if (rollResistanceBuffer.full())
    {
      std::vector<double> rollResistanceCandidates;
      for (size_t i = 0; i < rollResistanceBuffer.size(); i++)
        rollResistanceCandidates.push_back(rollResistanceBuffer[i]);

      std::sort(rollResistanceCandidates.begin(), rollResistanceCandidates.end());
      if (rollResistanceCandidates.size() % 2 == 0)
      {
        size_t index = (size_t)std::floor(rollResistanceCandidates.size() / 2);
        cognitionState.ballStatus.rollResistance = (rollResistanceCandidates[index] + rollResistanceCandidates[index - 1]) / 2;
      }
      else
      {
        size_t index = (size_t)std::floor(rollResistanceCandidates.size() / 2);
        cognitionState.ballStatus.rollResistance = rollResistanceCandidates[index];
      }
    }
  }
}

void CognitionMindfulness::kickDistanceSimulation(CognitionState& cognitionState)
{
  target = theMotionInfo.kickRequest.kickTarget;
  source = theBallSymbols.ballPositionField;

  DECLARE_DEBUG_DRAWING("representation:CognitionState:ballPredictionArea", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:CognitionState:ballPredictionCloud", "drawingOnField");
  POINTCLOUD(
      "representation:CognitionState:ballPredictionCloud", (int)pointCloud.size(), pointCloud, 45, 5, Drawings::solidPen, ColorRGBA(0, 0, 255, 25), Drawings::solidBrush, ColorRGBA(0, 0, 255, 25));
  CIRCLE("representation:CognitionState:ballPredictionCloud",
      cognitionState.ballStatus.ballPositionPrediction.x(),
      cognitionState.ballStatus.ballPositionPrediction.y(),
      45,
      5,
      Drawings::solidPen,
      ColorRGBA::violet,
      Drawings::solidBrush,
      ColorRGBA::violet);
  ARROW("representation:CognitionState:ballPredictionCloud", source.x(), source.y(), target.x(), target.y(), 5, Drawings::dashedPen, ColorRGBA::magenta);
  RECTANGLE("representation:CognitionState:ballPredictionArea", boundingBox[0], boundingBox[1], boundingBox[2], boundingBox[3], 15, Drawings::solidPen, ColorRGBA::cyan);
  if (!kickDataUpdated)
    return;

  pointCloud = kickSimulation(1000, target, source, cognitionState.ballStatus.swingDistance, cognitionState.ballStatus.swingTime);

  Vector2f ballPositionPrediction(0.f, 0.f);
  double maxX = DBL_MIN;
  double minX = DBL_MAX;
  double maxY = DBL_MIN;
  double minY = DBL_MAX;
  for (size_t i = 0; i < pointCloud.size(); i++)
  {
    ballPositionPrediction += pointCloud[i];
    if (maxX < pointCloud[i].x())
      maxX = pointCloud[i].x();
    if (maxY < pointCloud[i].y())
      maxY = pointCloud[i].y();
    if (minX > pointCloud[i].x())
      minX = pointCloud[i].x();
    if (minY > pointCloud[i].y())
      minY = pointCloud[i].y();
  }
  ballPositionPrediction /= (float)pointCloud.size();
  boundingBox[0] = minX;
  boundingBox[1] = minY;
  boundingBox[2] = maxX;
  boundingBox[3] = maxY;

  cognitionState.ballStatus.ballPositionPrediction = ballPositionPrediction;
  cognitionState.ballStatus.simulatedKickDistance = (ballPositionPrediction - source).norm();
  cognitionState.ballStatus.targetAngle = (target - source).angle();
  kickDataUpdated = false;
}

std::vector<Vector2f> CognitionMindfulness::kickSimulation(unsigned numberOfPredictions, Vector2f kickTarget, Vector2f ballPosition, double kickPathDistance, double kickTime, bool withoutDeviation)
{
  double dt = 0.1f;

  double gravity = 9.81;

  double footMass = 0.60372;

  double dragCoefficient = 0.45;
  double ballRadius = 0.05;
  double ballMass = 0.044;

  Angle alpha = 0_deg;

  double ballSpeedError = 0.071445197;
  Angle stdRollDeviation = 2.5_deg;
  Angle stdtargetAngleError = 8.1649658_deg;

  targetAngle = (kickTarget - ballPosition).angle();

  double footAcceleration = acceleration(kickPathDistance / 1000, kickTime / 1000, 0);
  double footSpeed = speed(footAcceleration, dt, 0);
  double ballSpeed = speedOfBall(footSpeed, footMass, ballMass);

  double gravityForce = force(ballMass, gravity);

  double normalForce = std::cos(alpha) * gravityForce;

  double rollResistanceForce = rollResistance * normalForce;

  std::vector<Vector2f> ballPositionCandidates;

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> ballSpeedWithError{ballSpeed, ballSpeedError};
  std::normal_distribution<> rollDeviation{0, stdRollDeviation};
  std::normal_distribution<> targetAngleError{0, stdtargetAngleError};

  for (size_t i = 0; i < numberOfPredictions; i++)
  {
    double s;
    if (!withoutDeviation)
      s = ballSpeedWithError(gen);
    else
      s = ballSpeed;

    double a = accelerationThruSpeed(s, dt, 0);
    double d = distance(a, dt, s, 0);

    Angle angle;
    if (!withoutDeviation)
      angle = targetAngle + (float)rollDeviation(gen) + (float)targetAngleError(gen);
    else
      angle = targetAngle;

    double distancePart = d;
    Vector2f newBallPosition = positionWithDeviation(ballPosition, distancePart * 1000, angle);
    while (distancePart > 0.001)
    {
      double dragForce = airDragBall(ballRadius, s, dragCoefficient);
      a = (force(a, ballMass) - rollResistanceForce - dragForce) / ballMass;
      s = speed(a, dt, s);
      d = distance(a, dt, s, d);
      distancePart = distance(a, dt, s, 0);
      newBallPosition = positionWithDeviation(newBallPosition, distancePart * 1000, angle);
    }
    ballPositionCandidates.push_back(newBallPosition);
  }

  return ballPositionCandidates;
}
