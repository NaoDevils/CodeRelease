#include "HeadControl.h"
#include <algorithm>
#include <functional>
#include "Representations/Perception/BodyContour.h"
#include "Tools/Range.h"
#include "Tools/Motion/InverseKinematic.h"

HeadControl::HeadControl()
{
  timeSinceTypeChange = 0;
  lastControlType = HeadControlRequest::soccer;
  targetQueue.reserve(10);
  lastPOI = Vector2f(1000, 0);
  lastPOIRel = Vector2f(1000, 0);
  localHeadAngleRequest.pan = 0;
  localHeadAngleRequest.tilt = 0;
  localHeadAngleRequest.speed = 40_deg;
  resetQueue();
  newPan = newTilt = 0;
  ballDistanceWasCritical = false;
  ballWasLeft = false;
  timeStampLastHeadPanDiff = 0;
  headPanDistances.clear();
  headPanDistances.fill(0);
  trackWithUpper = true;
  targetIsClose = false;
}

void HeadControl::resetQueue()
{
  calcAngles = true;
  targetQueue.clear();
  relativeAngleReached = false;
  timeStampTarget = theFrameInfo.time;
}

void HeadControl::handleLocalizeBall()
{
  headControlState = localizationOfBall;
  if (lastHeadControlState != localizationOfBall)
  {
    resetQueue();
    lastHeadControlState = localizationOfBall;
  }
  //lookAtPercept();  // this one looks at the last known ball position!!
  std::vector<Vector2f> pointsOfInterest;
  if (pointsOfInterest.size() < 2) // localization
  {
    pointsOfInterest.push_back(getBestFieldCrossingPoint());
  }
  if (pointsOfInterest.size() < 2)
    pointsOfInterest.push_back(Vector2f(0, 0));
  fillTargetQueue(true, pointsOfInterest);
}

void HeadControl::update(HeadAngleRequest& headAngleRequest)
{
  if (theFallDownState.state != FallDownState::upright)
    return;
  isLookingDown = false;
  timeForHeadMovement = 2000;

  std::vector<BallPerceptPosition>::iterator pos = falsifiedBallPositions.begin();
  while (pos != falsifiedBallPositions.end())
  {
    if (theFrameInfo.getTimeSince(pos->timeStampCreated) > timeToKeepBallPerceptPosition)
      falsifiedBallPositions.erase(pos);
    else
      pos++;
  }

  if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 2 * 35)
    ballWasLeft = theBallModel.lastPerception.angle() > 0;

  if (theFrameInfo.getTimeSince(timeStampLastHeadPanDiff) > 400)
  {
    headPanDistances.push_front(theJointSensorData.angles[Joints::headYaw] - headPanDistances[0]);
    timeStampLastHeadPanDiff = theFrameInfo.time;
  }

  if (theHeadControlRequest.controlType != lastControlType)
  {
    timeSinceTypeChange = 0;
  }

  switch (theHeadControlRequest.controlType)
  {
  case HeadControlRequest::ball:
  {
    resetQueue();
    PointOfInterest newPOI;
    const BallState& ballState = theBallModel.estimate;

    Vector2f pointOnField;
    pointOnField.x() = static_cast<float>(ballState.position.x() + ballState.velocity.x() / 2);
    pointOnField.y() = static_cast<float>(ballState.position.y() + ballState.velocity.y() / 2);

    newPOI.maxSpeed = headSpeedMax;
    newPOI.timeAtTarget = 0;
    newPOI.pointOnField = Transformation::robotToField(theRobotPose, pointOnField);
    newPOI.waitForReached = false;
    newPOI.relativeAngle = ballState.position.angle();
    newPOI.maxOverShoot = 15_deg;
    timeStampTarget = theFrameInfo.time;
    targetReached = false;
    targetQueue.push_back(newPOI);
  }
  break;
  case HeadControlRequest::direct:
    break;
  case HeadControlRequest::localize:
    headControlState = localization;
    sweepField(45_deg, -45_deg, 3000, false, headSpeedOpt);
    break;
  case HeadControlRequest::opponents:
    lookAtRobots();
    break;
  case HeadControlRequest::soccer:
    calculatePointsOfInterest();
    break;
  case HeadControlRequest::localizeBall:
    //handleLocalizeBall(); TODO: overhaul
    headControlState = localizationOfBall;
    sweepField(45_deg, -45_deg, 3000, false, 0.75f * headSpeedOpt);
    break;
  default:
    calculatePointsOfInterest();
  }
  lastControlType = theHeadControlRequest.controlType;

  if (calcAngles)
    calcHeadAngles();
  moveHead();
  headAngleRequest = localHeadAngleRequest;

  // drawing of the ball model in the field view
  DEBUG_DRAWING("module:HeadControl:targetQueue", "drawingOnField")
  {
    CROSS("module:HeadControl:targetQueue",
        lastPOI.x(),
        lastPOI.y(),
        60,
        10, // pen width
        Drawings::solidPen,
        ColorRGBA::black);

    for (size_t i = 0; i < targetQueue.size(); i++)
    {
      CROSS("module:HeadControl:targetQueue",
          targetQueue.at(i).pointOnField.x(),
          targetQueue.at(i).pointOnField.y(),
          60,
          10, // pen width
          Drawings::solidPen,
          ColorRGBA::black);
      DRAWTEXT("module:HeadControl:targetQueue", targetQueue.at(i).pointOnField.x(), targetQueue.at(i).pointOnField.y() + 10, 60, ColorRGBA::red, targetQueue.size() - i);
      if (i > 0)
        ARROW("module:HeadControl:targetQueue",
            targetQueue.at(i - 1).pointOnField.x(),
            targetQueue.at(i - 1).pointOnField.y(),
            targetQueue.at(i).pointOnField.x(),
            targetQueue.at(i).pointOnField.y(),
            10,
            Drawings::solidPen,
            ColorRGBA::black);
    }
  }
}

void HeadControl::lookAtPointOnField(const Vector2f& point, Angle speed, bool waitForReached, int timeAtTarget)
{
  if (theFallDownState.state == FallDownState::onGround)
    return;
  Angle angleToPointRelative = Transformation::fieldToRobot(theRobotPose, point).angle();
  resetQueue();
  PointOfInterest newPOI;
  newPOI.maxSpeed = speed;
  newPOI.timeAtTarget = timeAtTarget;
  newPOI.pointOnField = point;
  newPOI.waitForReached = waitForReached;
  newPOI.relativeAngle = angleToPointRelative;
  timeStampTarget = theFrameInfo.time;
  targetReached = false;
  targetQueue.push_back(newPOI);
}

void HeadControl::lookAtPercept() // TODO use BallPercept instead of BallModel
{
  resetQueue();
  PointOfInterest newPOI;
  newPOI.maxSpeed = headSpeedMax;
  newPOI.timeAtTarget = 0;
  newPOI.pointOnField = Vector2f((theBallModel.lastPerception.x() + theBallSymbols.ballVelocityRelativeWOPreview.x() / 2),
      (theBallModel.lastPerception.y() + theBallSymbols.ballVelocityRelativeWOPreview.y() / 2));
  newPOI.pointOnField = Transformation::robotToField(theRobotPose, newPOI.pointOnField);
  newPOI.waitForReached = false;
  newPOI.relativeAngle = theBallModel.lastPerception.angle();
  timeStampTarget = theFrameInfo.time;
  targetReached = false;
  targetQueue.push_back(newPOI);
}

void HeadControl::lookAtBall(bool lookAtPercepts)
{
  if (lookAtPercepts)
  {
    lookAtPercept();
    return;
  }
  // TODO: need timeSinceBallCouldBeSeen!
  const int timeSinceBallWasSeen = theBallSymbols.timeSinceLastSeen;

  // TODO: use predicted position from ball symbols
  if (!theBallSymbols.ballFoundAfterDropIn)
  {
    Angle angle = 75_deg;
    headControlState = ballSweepWide;
    sweepField(angle, -angle, 1000, true, headSpeedOpt);
    return;
  }

  if (theBallSymbols.ballProbablyCloseButNotSeen)
  {
    handleBallLost();
    return;
  }
  else
    wasBallLost = false;

  // TODO: check
  if (theBallSymbols.ballHitsMe && timeSinceBallWasSeen > 300 && timeSinceBallWasSeen < 2000)
  {
    resetQueue();
    PointOfInterest newPOI(
        Transformation::robotToField(theRobotPose, theBallSymbols.ballPositionRelativeWOPreview), headSpeedMax, 0, false, theBallSymbols.ballPositionRelativeWOPreview.angle());
    timeStampTarget = theFrameInfo.time;
    targetReached = false;
    targetQueue.push_back(newPOI);
    return;
  }

  float ballDistance = theBallSymbols.ballPositionRelativeWOPreview.norm();
  float angleToBall = theBallSymbols.ballPositionRelativeWOPreview.angle();
  if (std::abs(angleToBall) < 0.4f && ballDistance < 2500 && ballDistance > 400 && timeSinceBallWasSeen > 500 && theBallSymbols.ballVelocityRelative.norm() < 100)
  {
    headControlState = ballSweep;
    sweepField(angleToBall + 20_deg, angleToBall - 20_deg, ballLostSweepFieldDistance, true, headSpeedOpt);
    return;
  }

  if (theBallSymbols.timeSinceLastSeenByTeam > 3000)
  {
    if (ballDistance < 1000)
    {
      headControlState = ballSweepWide;
      sweepField(60_deg, -60_deg, ballLostSweepFieldDistance, false, headSpeedOpt, false);
      return;
    }
    else
    {
      headControlState = ballSweep;
      sweepField(20_deg, -20_deg, ballLostSweepFieldDistance, true, headSpeedOpt);
      return;
    }
  }

  resetQueue();
  PointOfInterest newPOI;
  newPOI.maxSpeed = headSpeedMax;
  newPOI.timeAtTarget = 0;
  newPOI.pointOnField = Vector2f((theBallSymbols.ballPositionRelativeWOPreview.x() + theBallSymbols.ballVelocityRelativeWOPreview.x() / 2),
      (theBallSymbols.ballPositionRelativeWOPreview.y() + theBallSymbols.ballVelocityRelativeWOPreview.y() / 2));
  newPOI.pointOnField = Transformation::robotToField(theRobotPose, newPOI.pointOnField);
  newPOI.waitForReached = false;
  newPOI.relativeAngle = angleToBall;
  timeStampTarget = theFrameInfo.time;
  targetReached = false;
  targetQueue.push_back(newPOI);
}

void HeadControl::handleBallLost()
{
  if (!wasBallLost)
  {
    wasBallLost = true;
    timeStampBallLost = theFrameInfo.time;
  }
  /*if (theFrameInfo.getTimeSince(timeStampBallLost) < 500)
  {
    resetQueue();
    PointOfInterest newPOI;
    newPOI.maxSpeed = headSpeedMax;
    newPOI.timeAtTarget = 0;
    const BallState &ballState = myBallModel.lastPerception;
    newPOI.pointOnField = Vector2f((float)(ballState.position.x()),
      (float)(ballState.position.y()));
    newPOI.pointOnField = Geometry::relative2FieldCoord(theRobotPose,newPOI.pointOnField);
    newPOI.waitForReached = false;
    timeStampTarget = theFrameInfo.time;
    targetReached = false;
    targetQueue.push_back(newPOI);
  }
  else*/
  {
    headControlState = ballLost;
    sweepField(60_deg, -60_deg, ballLostSweepFieldDistance, false, 0.5f * (headSpeedOpt + headSpeedMax), true, true);
  }
}

void HeadControl::calcHeadAngles()
{
  Angle headPan = 0_deg;
  Angle headTilt = theJointCalibration.joints[Joints::headPitch].maxAngle;

  if (targetQueue.empty())
    return;

  //const float distanceToTarget = (lastPOIRel-Transformation::fieldToRobot(theRobotPose,targetQueue.back().pointOnField)).norm();
  //const Angle distanceToTargetAngle = std::abs(Angle::normalize(Transformation::fieldToRobot(theRobotPose,lastPOI).angle()-Transformation::fieldToRobot(theRobotPose,targetQueue.back().pointOnField).angle()));
  //const float distanceDifference = std::abs(lastPOI.norm()-targetQueue.back().pointOnField.norm());

  Vector2f latestPOI = targetQueue.back().pointOnField;

  Vector2f targetRel = Transformation::fieldToRobot(theRobotPose, latestPOI);
  if ((lastPOIRel - targetRel).norm() < 50)
    return;
  lastPOIRel = targetRel;
  lastPOI = latestPOI;

  ////Calculate the head-pan:
  //headPan = std::atan2(targetRel.y(), targetRel.x());

  // here (i.e. Behavior) all angles are in degree
  float targetDistance = targetRel.norm();
  if (targetDistance > 650)
  {
    // if target is far away, track with upper camera
    trackWithUpper = true;
  }
  else if (targetDistance < 550)
  {
    // if target is near, track with lower camera
    trackWithUpper = false;
  }

  Vector3f target(targetRel.x(), targetRel.y(), 0);
  // TODO: why is bh method not really working as expected (centering object)?
  //const CameraMatrix &cm = trackWithUpper ? theCameraMatrixUpper : theCameraMatrix;
  //Angle tiltAngleToTarget2 = std::atan2(cm.translation.z(), target.norm());
  //InverseKinematic::calcHeadJoints(target, pi_2-pi_8, theRobotDimensions, !trackWithUpper, panTilt, theCameraCalibration);
  //headTilt = toDegrees(panTilt.y());
  //targetIsClose = (targetDistance < (targetIsClose ? 2500 : 2000));
  //if (theBehaviorData.role == BehaviorData::keeper && targetIsClose)
  //  headTilt = std::min<Angle>(tiltAngleToTarget - theRobotDimensions.getTiltNeckToCamera(!trackWithUpper) + 5_deg, theJointCalibration.joints[Joints::headPitch].maxAngle);
  //else
  //  headTilt = tiltAngleToTarget - theRobotDimensions.getTiltNeckToCamera(!trackWithUpper);

  Vector3f hip2Target = theTorsoMatrix.inverse() * target;
  Vector2f panTilt;
  InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, !trackWithUpper, panTilt, theCameraCalibration);
  headPan = panTilt.x();
  headTilt = panTilt.y();

  // clip angles
  Range<float> rangePan(theJointCalibration.joints[Joints::headYaw].minAngle, theJointCalibration.joints[Joints::headYaw].maxAngle);
  headPan = rangePan.limit(headPan);

  //Range<float> rangeTilt(theJointCalibration.joints[Joints::headPitch].minAngle, theJointCalibration.joints[Joints::headPitch].maxAngle);
  Range<float> rangeTilt(minTiltAngle, theJointCalibration.joints[Joints::headPitch].maxAngle);
  headTilt = rangeTilt.limit(headTilt);

  // no minimal movement
  if (std::abs(theJointSensorData.angles[Joints::headYaw] - headPan) < minHeadMovementPan)
    headPan = theJointSensorData.angles[Joints::headYaw];

  newPan = headPan;
  newTilt = headTilt;
  calcAngles = false;
}

void HeadControl::calculatePointsOfInterest()
{
  if (theRobotInfo.penalty != PENALTY_NONE)
  {
    lookDown();
    return;
  }

  switch (theGameInfo.state)
  {
  case STATE_INITIAL:
    lookDown();
    break;
  case STATE_READY:
    handleReadyState();
    break;
  case STATE_SET:
    if (theGameInfo.setPlay == SET_PLAY_NONE && (theGameInfo.gamePhase == GAME_PHASE_NORMAL || theGameInfo.gamePhase == GAME_PHASE_OVERTIME))
    {
      headControlState = set;
      sweepField(45_deg, -45_deg, theRobotPose.translation.norm(), true, headSpeedOpt, false);
    }
    else
    {
      if (theRobotPose.validity > 0.5f && theRobotPose.translation.x() > -1500 && theBallSymbols.ballWasSeen)
        lookAtBall(false);
      else
      {
        headControlState = set;
        sweepField(45_deg, -45_deg, theRobotPose.translation.norm(), true, headSpeedOpt, false);
      }
    }
    break;
  case STATE_PLAYING:
    handlePlayingState();
    break;
  case STATE_FINISHED:
    lookDown();
    break;
  default:
    lookDown();
    break;
  }
}

void HeadControl::handleReadyState()
{
  const bool inGoto = theMotionRequest.walkRequest.requestType == WalkRequest::destination && theMotionRequest.motion == MotionRequest::walk;
  std::vector<Vector2f> pointsOfInterest;
  if (inGoto)
  {
    if (targetQueue.empty())
    {
      addPointsOnPath(pointsOfInterest);

      // path is covered, now either look for ball or aid own localization
      if (pointsOfInterest.size() < 2)
      {
        // TODO.. obstacles?
        pointsOfInterest.push_back(getBestFieldCrossingPoint());
      }
    }
  }
  else
  {
    headControlState = ready;
    sweepField(45_deg, -45_deg, 3000, true, headSpeedOpt, false, false);
    return;
  }

  if (targetQueue.empty() && pointsOfInterest.size() > 1)
  {
    Range<float> validRange = Range<float>(-30_deg, 30_deg);
    float relAngle0 = Transformation::fieldToRobot(theRobotPose, pointsOfInterest[0]).angle();
    relAngle0 = validRange.limit(relAngle0);
    float relAngle1 = Transformation::fieldToRobot(theRobotPose, pointsOfInterest[1]).angle();
    relAngle0 = validRange.limit(relAngle1);
    // there are at least 2 points now to look at
    if (sgn(theJointSensorData.angles[Joints::headYaw]) == sgn(relAngle0))
    {
      targetQueue.push_back(PointOfInterest(pointsOfInterest[0], headSpeedOpt, 0, !inGoto, relAngle0));
      targetQueue.push_back(PointOfInterest(pointsOfInterest[1], headSpeedOpt, 0, !inGoto, relAngle1));
    }
    else
    {
      targetQueue.push_back(PointOfInterest(pointsOfInterest[1], headSpeedOpt, 0, !inGoto, relAngle1));
      targetQueue.push_back(PointOfInterest(pointsOfInterest[0], headSpeedOpt, 0, !inGoto, relAngle0));
    }
  }
  else
  {
    //should not happen
  }
}

void HeadControl::fillTargetQueue(const bool waitForReached, std::vector<Vector2f> pointsOfInterest)
{
  if (targetQueue.empty() && pointsOfInterest.size() > 1)
  {
    float relAngle0 = Transformation::fieldToRobot(theRobotPose, pointsOfInterest[0]).angle();
    float relAngle1 = Transformation::fieldToRobot(theRobotPose, pointsOfInterest[1]).angle();
    if (waitForReached)
    {
      // TODO: use this: float angleToDestination = theMotionRequest.walkRequest.request.translation.angle();
      // TODO: make sure to keep target in sight
      relAngle0 = std::min(0.75f, std::abs(relAngle0)) * sgn(relAngle0);
      relAngle1 = std::min(0.75f, std::abs(relAngle1)) * sgn(relAngle1);
    }
    // there are at least 2 points now to look at
    float maxOverShoot = waitForReached ? 10_deg : 20_deg;
    if (sgn(theJointSensorData.angles[Joints::headYaw]) == sgn(relAngle0))
    {
      targetQueue.push_back(PointOfInterest(pointsOfInterest[0], headSpeedOpt, 0, !waitForReached, relAngle0, maxOverShoot));
      targetQueue.push_back(PointOfInterest(pointsOfInterest[1], headSpeedOpt, 0, !waitForReached, relAngle1, maxOverShoot));
    }
    else
    {
      targetQueue.push_back(PointOfInterest(pointsOfInterest[1], headSpeedOpt, 0, !waitForReached, relAngle1, maxOverShoot));
      targetQueue.push_back(PointOfInterest(pointsOfInterest[0], headSpeedOpt, 0, !waitForReached, relAngle0, maxOverShoot));
    }
  }
  else if (targetQueue.empty())
  {
    ASSERT(false);
    //should not happen
  }
}

void HeadControl::handlePlayingState()
{
  // TODO HeadControl doesnt line up with body control!!! In BallchaserProvider waitInOwnSetPlay

  // if we are ballchaser, have reached our position, we are in our own set play and we have still time remaining
  // -> look around
  if (theBehaviorData.playerNumberToBall == theRobotInfo.number && theSpeedInfo.speed == Pose2f(0, Vector2f::Zero()) && theGameSymbols.ownKickOff
      && (theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_GOAL_KICK || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK))
  {
    sweepField(45_deg, -45_deg, 3000, false, headSpeedOpt);
    return;
  }

  //int timeSinceBallWasSeen = (int)theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  if ((fielderLookAtPercepts || goalieLookAtPercepts) && headControlState != verifyBall)
    perceptToVerify = Transformation::robotToField(theRobotPose, theBallModel.lastPerception);

  if (theBehaviorData.role == BehaviorData::keeper || theBehaviorData.role == BehaviorData::replacementKeeper)
  {
    handleGoalie();
    return;
  }

  lastRobotState = robotState;

  // look at ball, if near
  const float ballDistance = theBallSymbols.ballPositionRelativeWOPreview.norm();

  const float angleToBall = theBallSymbols.ballPositionRelativeWOPreview.angle();
  const bool inGoto = theMotionRequest.walkRequest.requestType == WalkRequest::destination && theMotionRequest.motion == MotionRequest::walk
      && theMotionRequest.walkRequest.request.translation.norm() - (robotState == gotoPoint ? 100.f : 0.f) > gotoStateMinDistance;
  const bool inKick = theMotionRequest.motion == MotionRequest::kick || (theMotionRequest.motion == MotionRequest::walk && ballDistance < 300 && theBallSymbols.ballWasSeen);
  const bool inGotoBall = theBehaviorData.soccerState == BehaviorData::controlBall;

  robotState = (inGotoBall || inKick) ? gotoBall : (inGoto ? gotoPoint : waiting);
  if (lastRobotState != robotState)
    resetQueue();
  std::vector<Vector2f> pointsOfInterest;

  // verify percepts if no useful ball model and percept is far away so few percepts are expected
  if (fielderLookAtPercepts && theBallModel.validity < maxBallModelValForPerceptCheck && theRemoteBallModel.validity < maxBallModelValForPerceptCheck && // if a ball model is present, do not check percepts
      (theBallPercept.status == BallPercept::seen || headControlState == verifyBall) && // start verification immediately
      Transformation::fieldToRobot(theRobotPose, perceptToVerify).norm() > minDistanceOfBallPerceptToVerify && // min distance needed, close percepts should be seen more often
      !(headControlState == verifyBall && theFrameInfo.getTimeSince(timeVerifyStarted) > timeToFalsifyPercept) && //stay in this state only for a short time to verify/falsify percept
      !isPerceptOnNoBallPosition()) // make sure not to check falsified positions twice
  {
    if (headControlState != verifyBall)
    {
      timeVerifyStarted = theFrameInfo.time;
      headControlState = verifyBall;
    }
    lookAtBall(true);
    return;
  }
  if (headControlState == verifyBall)
  {
    BallPerceptPosition pos;
    pos.position = perceptToVerify;
    pos.timeStampCreated = theFrameInfo.time;
  }

  switch (robotState)
  {
  case gotoBall:
    lookAtBall(false);
    return;
    break;
  case gotoPoint:
    if (theBehaviorData.soccerState == BehaviorData::searchForBall || thePositioningSymbols.inBallSearch)
    {
      if (pointsOfInterest.size() < 2) // localization
      {
        pointsOfInterest.push_back(getBestFieldCrossingPoint());
      }
      if (pointsOfInterest.size() < 2)
      {
        Angle angleBallSearch = 60_deg;
        if (!pointsOfInterest.empty() && Transformation::fieldToRobot(theRobotPose, pointsOfInterest[0]).angle() > 0)
          angleBallSearch *= -1.f;
        pointsOfInterest.push_back(Transformation::robotToField(theRobotPose, Vector2f(2500.f, 0.f).rotate(angleBallSearch)));
      }
    }
    if (targetQueue.empty())
    {
      addPointsOnPath(pointsOfInterest);

      // path is covered, now either look for ball or aid own localization
      if (pointsOfInterest.size() < 2)
      {
        // TODO..
        if ((std::abs(angleToBall) < theCameraInfo.openingAngleWidth * 0.75f && ballDistance < 5000) || theBallSymbols.timeSinceLastSeenByTeam > 5000)
        {
          Angle angleBallSearch = 60_deg;
          if (theJointSensorData.angles[Joints::headYaw] > 0)
            angleBallSearch *= -1.f;
          pointsOfInterest.push_back(Transformation::robotToField(theRobotPose, Vector2f(ballDistance, 0).rotate(angleBallSearch)));
        }
        if (pointsOfInterest.size() < 2) // localization
        {
          pointsOfInterest.push_back(getBestFieldCrossingPoint());
        }
        if (pointsOfInterest.size() < 2)
          pointsOfInterest.push_back(Vector2f(0, 0));
      }
    }
    break;
  case waiting:
    if (ballDistance < criticalBallDistance || theBallSymbols.ballVelocityRelative.norm() > 100 || theRemoteBallModel.velocity.norm() > 150)
    {
      lookAtBall(false);
      return;
    }
    else
    {
      lookAtRobots();
      return;
    }
    break;
  default:
    sweepField(60_deg, -60_deg, 1000, true, headSpeedOpt);
    break;
  }

  fillTargetQueue(inGoto, pointsOfInterest);
}

Vector2f HeadControl::getBestFieldCrossingPoint()
{
  std::vector<Vector2f> fieldCrossingPoints;
  fieldCrossingPoints.push_back(Vector2f(0, 0));
  fieldCrossingPoints.push_back(Vector2f(0, theFieldDimensions.yPosRightSideline));
  fieldCrossingPoints.push_back(Vector2f(0, theFieldDimensions.yPosLeftSideline));

  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoalArea));

  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoalArea));

  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoalArea));

  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea));
  fieldCrossingPoints.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea));

  float minAngleToFieldCrossingPoint = std::abs(Transformation::fieldToRobot(theRobotPose, fieldCrossingPoints[0]).angle());
  unsigned int minIndex = 0;

  for (unsigned int i = 1; i < fieldCrossingPoints.size(); i++)
  {
    float angleToFCP = std::abs(Transformation::fieldToRobot(theRobotPose, fieldCrossingPoints[i]).angle());
    if (angleToFCP < minAngleToFieldCrossingPoint)
    {
      minIndex = i;
      minAngleToFieldCrossingPoint = angleToFCP;
    }
  }

  return fieldCrossingPoints[minIndex];
}

void HeadControl::handleGoalie()
{
  float ballDistance = theBallSymbols.ballPositionRelativeWOPreview.norm();
  float ballSpeed = std::max(theBallSymbols.ballVelocityRelativeWOPreview.norm(), theRemoteBallModel.velocity.norm());

  Vector2f perceptToVerifyRelative = Transformation::fieldToRobot(theRobotPose, perceptToVerify);

  if (goalieLookAtPercepts && (theBallPercept.status == BallPercept::seen || headControlState == verifyBall)
      && (theBallSymbols.ballPositionRelativeWOPreview - perceptToVerifyRelative).norm() > 500 && //if percepts fits ball model, do not care
      (theBallModel.lastPerception - perceptToVerifyRelative).norm() < 500 && // not a newer percept?
      !(!theBallSymbols.useLocalBallModel && theRemoteBallModel.validity > 0.7 && theRemoteBallModel.position.x() > -1000) && //if a good team ball model is far away, dont care
      !(theBallSymbols.useLocalBallModel && theBallModel.validity > 0.7) && //if local ball model is fine, dont care
      !(headControlState == verifyBall && theFrameInfo.getTimeSince(timeVerifyStarted) > timeToFalsifyPercept) && //stay in this state only for a short time to verify/falsify percept
      !isPerceptOnNoBallPosition()) //if percept was falsified around this position, dont care
  {
    if (headControlState != verifyBall)
    {
      timeVerifyStarted = theFrameInfo.time;
      headControlState = verifyBall;
    }
    lookAtBall(true);
    return;
  }
  if (headControlState == verifyBall)
  {
    BallPerceptPosition pos;
    pos.position = perceptToVerify;
    pos.timeStampCreated = theFrameInfo.time;
  }
  // ball should be near, but is not seen
  if ((theBallSymbols.timeSinceLastSeenByTeam > 5000 && theBallSymbols.ballPositionField.x() < theFieldDimensions.xPosOwnPenaltyArea) || theBallSymbols.timeSinceLastSeenByTeam > 15000)
  {
    if (headControlState != keeperSweepWide)
      timeStampSwitchedToNewState = theFrameInfo.time;
    headControlState = keeperSweepWide;
    //OUTPUT_TEXT("Keeper sweep wide");
    int timeSinceState = theFrameInfo.getTimeSince(timeStampSwitchedToNewState);
    if (timeSinceState < 3000)
      sweepField(80_deg, -80_deg, 250, false, headSpeedOpt, false, false); //
    else
    {
      int secondsInState = (timeSinceState - 3000) / 1000;
      Angle targetAngle = -100_deg + (secondsInState % 6) * 40_deg;
      Vector2f pRobot(250, 0);
      pRobot.rotate(targetAngle);
      Vector2f pField = Transformation::robotToField(theRobotPose, pRobot);
      lookAtPointOnField(pField, headSpeedMax, true, 500);
    }
    return;
  }
  timeStampSwitchedToNewState = theFrameInfo.time;

  // Check if we are in goto
  const bool inGoto = theMotionRequest.walkRequest.requestType == WalkRequest::destination && theMotionRequest.motion == MotionRequest::walk;
  // RobotPose to ball
  Angle meToBall = theBallSymbols.ballPositionRelative.angle();
  // Ball behind me?
  bool ballBehindMe = std::abs(meToBall) > pi_2;

  if (inGoto && ballBehindMe && theBallSymbols.ballPositionRelative.norm() < 1500)
  {
    sweepField(45_deg, -45_deg, 3000, false, headSpeedOpt); //
  }
  // if ball is not moving fast and far away, try to stabilize localization, TO TEST!
  else if (ballSpeed < 100 && theBallSymbols.timeSinceLastSeenByTeam < 3000)
  {
    headControlState = keeperSweepBall;
    //OUTPUT_TEXT("Keeper sweep ball far");
    Angle angleToBall = theBallSymbols.ballPositionRelativeWOPreview.angle();
    float openingAngleFactor = ballDistance > safeBallDistanceGoalie ? 1.f
        : ballDistance < criticalBallDistanceGoalie
        ? 0.f
        : (ballDistance - criticalBallDistanceGoalie) / (safeBallDistanceGoalie - criticalBallDistanceGoalie);
    Angle openingAnglePerSide = (openingAngleFactor * (goalieSweepFieldFullAngle - goalieSweepFieldMinAngleWhenBallNotMoving) + goalieSweepFieldMinAngleWhenBallNotMoving) / 2;
    sweepField(angleToBall + openingAnglePerSide, angleToBall - openingAnglePerSide, theBallSymbols.ballPositionRelativeWOPreview.norm(), true, headSpeedSafe, true, true);
  }
  // if ball is moving fast, look at it
  else if (ballSpeed > 50 || (ballDistance < criticalBallDistanceGoalie && theBallSymbols.timeSinceLastSeen < 3000))
  {
    //OUTPUT_TEXT("Keeper look at ball");
    //resetQueue();
    lookAtBall(false);
  }
  // default : sweep field around ball
  else
  {
    headControlState = keeperSweepBall;
    //OUTPUT_TEXT("Keeper sweep ball near");
    float angleToBall = theBallSymbols.ballPositionRelativeWOPreview.angle();
    sweepField(
        angleToBall + goalieSweepFieldMinAngleWhenBallNotMoving / 2, angleToBall - goalieSweepFieldMinAngleWhenBallNotMoving / 2, theBallSymbols.ballPositionRelativeWOPreview.norm(), true, headSpeedSafe, true, false);
  }
  return;
}

void HeadControl::addPointsOnPath(std::vector<Vector2f>& points)
{
  Vector2f target(theMotionRequest.walkRequest.request.translation);
  Vector2f meToTarget(Transformation::robotToField(theRobotPose, target) - theRobotPose.translation);
  std::vector<Pose2f> robotsOnPath;
  float meToTargetAngle = meToTarget.angle();
  bool robotFound = false;
  float closestRobotDistance = 100000.0;

  for (int i = 0; i < (int)theRobotMap.robots.size(); i++)
  {
    const Vector2f& otherRobot(theRobotMap.robots[i].pose.translation);
    Vector2f meToRobot(otherRobot - theRobotPose.translation);
    if (meToRobot.norm() < 3000 && std::abs(Angle::normalize(meToRobot.angle() - meToTargetAngle)) < pi_4 && sgn(meToRobot.angle()) != sgn(theJointSensorData.angles[Joints::headYaw]))
    {
      float robotDist = otherRobot.norm();
      robotsOnPath.push_back(theRobotMap.robots[i].pose);
      if (robotDist < closestRobotDistance)
      {
        closestRobotDistance = robotDist;
        robotFound = true;
      }
    }
  }
  if (!robotFound)
  {
    float angleToLastPOI = Transformation::fieldToRobot(theRobotPose, lastPOI).angle();
    if (sgn(angleToLastPOI) >= 0)
      points.push_back(Transformation::robotToField(theRobotPose, Vector2f(1500, 0).rotate(-pi_4 * 0.8f)));
    else
      points.push_back(Transformation::robotToField(theRobotPose, Vector2f(1500, 0).rotate(pi_4 * 0.8f)));
  }
  else
  {

    std::sort(robotsOnPath.begin(), robotsOnPath.end(), sortRobots(this));
    for (unsigned int i = 0; i < robotsOnPath.size(); i++)
      points.push_back(robotsOnPath[i].translation);
  }
}

void HeadControl::lookAtRobots()
{
  if (targetQueue.empty() || lastControlType != theHeadControlRequest.controlType)
  {
    const float maxDist = 3000;
    std::vector<Pose2f> poses;
    std::vector<float> angles;

    for (auto& robot : theRobotMap.robots)
    {
      Vector2f targetRel = Transformation::fieldToRobot(theRobotPose, robot.pose.translation);
      float newAngle = targetRel.angle();
      if (targetRel.norm() < maxDist && std::abs(targetRel.angle()) < pi_2)
      {
        float minAngleDiff = pi;
        for (int i = 0; i < (int)angles.size(); i++)
        {
          minAngleDiff = std::min<float>(std::abs(Angle::normalize(angles[i] - newAngle)), minAngleDiff);
        }
        if (minAngleDiff > 20_deg)
        {
          poses.push_back(Pose2f(robot.pose));
          angles.push_back(newAngle);
        }
      }
      if (poses.size() >= 3)
        break;
    }
    if (poses.empty())
    {
      poses.emplace_back(0, Transformation::robotToField(theRobotPose, Vector2f(3000, 3000)));
      poses.emplace_back(0, Transformation::robotToField(theRobotPose, Vector2f(1000, -1000)));
    }
    else if (poses.size() == 1)
    {
      poses.emplace_back(0, Transformation::robotToField(theRobotPose, Vector2f(1000.f, sgn(Transformation::fieldToRobot(theRobotPose, poses[0].translation).y()) * -1000.f)));
    }
    resetQueue();
    std::sort(poses.begin(), poses.end(), sortRobots(this));
    for (int i = 0; i < (int)poses.size(); i++)
    {
      PointOfInterest newPOI;
      newPOI.maxSpeed = headSpeedOpt;
      newPOI.pointOnField = poses[i].translation;
      newPOI.timeAtTarget = 500;
      newPOI.waitForReached = true;
      newPOI.relativeAngle = Transformation::fieldToRobot(theRobotPose, poses[i].translation).angle();
      newPOI.maxOverShoot = 25_deg;
      targetQueue.push_back(newPOI);
      if (targetQueue.size() > 4)
        break;
    }
    /*sortPoints s(this);
    std::sort(targetQueue.begin(), targetQueue.end(), s);*/
  }
}

void HeadControl::lookDown()
{
  isLookingDown = true;
}

void HeadControl::moveHead()
{
  const bool direct = (theHeadControlRequest.controlType == HeadControlRequest::direct || targetQueue.empty());
  if (theFallDownState.state == FallDownState::onGround)
    return;
  // from headmotionengine
  Angle lastPan = theJointSensorData.angles[Joints::headYaw];
  if (lastPan == JointAngles::off)
    lastPan = newPan;
  Angle lastTilt = theJointSensorData.angles[Joints::headPitch];
  if (lastTilt == JointAngles::off)
    lastTilt = newTilt;
  const Vector2f lastPosition(lastPan, lastTilt);
  const Vector2f target(((newPan == JointAngles::off) ? lastPan : newPan), ((newTilt == JointAngles::off) ? lastTilt : newTilt));
  const float distanceToTarget = (lastPosition - target).norm();

  localHeadAngleRequest.pan = newPan;
  localHeadAngleRequest.tilt = newTilt;
  localHeadAngleRequest.speed = direct ? headSpeedMax : targetQueue.back().maxSpeed;

  if (!direct && !isLookingDown)
  {
    float relativeAngleDiff = std::abs(targetQueue.back().relativeAngle - lastPan);
    relativeAngleReached = !relativeAngleReached && sgn(targetQueue.back().relativeAngle - lastPan) != sgn(newPan - lastPan);

    // TODO: check overshoot and reached stuff
    if ((distanceToTarget < 5_deg || (relativeAngleDiff > targetQueue.back().maxOverShoot && relativeAngleReached)
            || (targetQueue.back().waitForReached == false && theFrameInfo.getTimeSince(timeStampTarget) > timeForHeadMovement) || theFrameInfo.getTimeSince(timeStampTarget) > 5000
            || (std::abs(headPanDistances.sum()) < 20_deg && theFrameInfo.getTimeSince(timeStampTarget) > 1000))
        && !targetReached)
    {
      timeStampTarget = theFrameInfo.time;
      targetReached = true;
    }
    if (targetReached && ((theFrameInfo.getTimeSince(timeStampTarget) > targetQueue.back().timeAtTarget) || targetQueue.back().waitForReached == false))
    {
      targetQueue.pop_back();
      targetReached = false;
      relativeAngleReached = false;
      calcAngles = true;
    }
  }
  else
  {
    localHeadAngleRequest.pan = isLookingDown ? 0_deg : theHeadControlRequest.pan;
    localHeadAngleRequest.tilt = isLookingDown ? theJointCalibration.joints[Joints::headPitch].maxAngle : theHeadControlRequest.tilt;
    localHeadAngleRequest.speed = headSpeedMax;
  }
}

void HeadControl::sweepField(const Angle leftAngle, const Angle rightAngle, const float distance, const bool forceSweepType, const Angle speed, bool sweepNearestFirst, bool waitForReached)
{
  if (theFallDownState.state == FallDownState::onGround)
    return;
  if (targetQueue.empty() || (lastControlType != theHeadControlRequest.controlType && !forceSweepType) || headControlState != lastHeadControlState)
  //|| robotState != lastRobotState) TODO: check this
  {
    lastHeadControlState = headControlState;
    resetQueue();
    bool isLeft = theJointSensorData.angles[Joints::headYaw] > 0;
    if (!sweepNearestFirst)
      isLeft = leftAngle > rightAngle;
    PointOfInterest newPOI, newPOI2;
    newPOI.maxSpeed = speed;
    newPOI.pointOnField = Vector2f(isLeft ? distance * std::cos(leftAngle) : distance * std::cos(rightAngle), isLeft ? distance * std::sin(leftAngle) : distance * std::sin(rightAngle));
    newPOI.pointOnField.x() = std::abs(newPOI.pointOnField.x());
    newPOI.relativeAngle = newPOI.pointOnField.angle();
    newPOI.pointOnField = Transformation::robotToField(theRobotPose, newPOI.pointOnField);
    newPOI.timeAtTarget = 100;
    newPOI.waitForReached = waitForReached;
    newPOI.maxOverShoot = (waitForReached ? 40_deg : 15_deg);

    newPOI2.maxSpeed = speed;
    newPOI2.pointOnField = Vector2f(isLeft ? distance * std::cos(rightAngle) : distance * std::cos(leftAngle), isLeft ? distance * std::sin(rightAngle) : distance * std::sin(leftAngle));
    newPOI2.pointOnField.x() = std::abs(newPOI2.pointOnField.x());
    newPOI2.relativeAngle = newPOI2.pointOnField.angle();
    newPOI2.pointOnField = Transformation::robotToField(theRobotPose, newPOI2.pointOnField);
    newPOI2.timeAtTarget = 100;
    newPOI2.waitForReached = waitForReached;
    newPOI2.maxOverShoot = (waitForReached ? 40_deg : 15_deg);
    targetQueue.push_back(newPOI);
    targetQueue.push_back(newPOI2);
  }
}

bool HeadControl::isPerceptOnNoBallPosition()
{
  for (auto& noPos : falsifiedBallPositions)
  {
    if ((noPos.position - Transformation::robotToField(theRobotPose, theBallModel.lastPerception)).norm() < 500)
      return true;
  }
  return false;
}

MAKE_MODULE(HeadControl, behaviorControl)
