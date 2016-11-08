#include "HeadControl2014.h"
#include <algorithm>
#include <functional>
#include "Representations/Perception/BodyContour.h"
#include "Tools/Range.h"
#include "Tools/Motion/InverseKinematic.h"

HeadControl2014::HeadControl2014()
{
  timeSinceTypeChange = 0;
  lastControlType = HeadControlRequest::soccer;
  targetQueue.reserve(10);
  lastPOI = Vector2f(1000,0);
  lastPOIRel = Vector2f(1000,0);
  localHeadAngleRequest.pan = 0;
  localHeadAngleRequest.tilt = 0;
  localHeadAngleRequest.speed = 40_deg;
  resetQueue();
  newPan = newTilt = 0;
  timeStampLastHeadPanDiff = 0;
  headPanDistances.clear();
  headPanDistances.fill(0);
  trackWithUpper = true;
  targetIsClose = false;
}

void HeadControl2014::resetQueue()
{
  targetQueue.clear();
  relativeAngleReached = false;
  if (Blackboard::getInstance().exists("FrameInfo"))
    timeStampTarget = theFrameInfo.time;
}

void HeadControl2014::update(HeadAngleRequest &headAngleRequest)
{
  DECLARE_DEBUG_DRAWING("module:HeadControl2014:wouldBeVisible","drawingOnField");
  
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
  
  if (theFrameInfo.getTimeSince(timeStampLastHeadPanDiff) > 400)
  {
    headPanDistances.push_front(theJointSensorData.angles[Joints::headYaw]-headPanDistances[0]);
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
      newPOI.maxSpeed = headSpeedMax;
      newPOI.timeAtTarget = 0;
      const BallState &ballState = theBallModel.estimate;
      newPOI.pointOnField = Vector2f((float)(ballState.position.x()+ballState.velocity.x()/2),
        (float)(ballState.position.y()+ballState.velocity.y()/2));
      newPOI.pointOnField = Transformation::robotToField(theRobotPose,newPOI.pointOnField);
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
    sweepField(-0.8f,0.8f,3000,false,headSpeedOpt);
    break;
  case HeadControlRequest::opponents:
    sweepField(-1.f, 1.f, 3000, false, headSpeedOpt);
    break;
  case HeadControlRequest::soccer:
    calculatePointsOfInterest();
    break;
  default:
    calculatePointsOfInterest();
  }
  lastControlType = theHeadControlRequest.controlType;
  calcHeadAngles();
  moveHead();
  headAngleRequest = localHeadAngleRequest;
}

void HeadControl2014::lookAtPointOnField(const Vector2f &point, Angle speed, bool waitForReached, int timeAtTarget)
{
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

void HeadControl2014::lookAtBall(bool lookAtPercept)
{
  if (lookAtPercept)
  {
    resetQueue();
    PointOfInterest newPOI;
    newPOI.maxSpeed = headSpeedMax;
    newPOI.timeAtTarget = 0;
    newPOI.pointOnField = theBallModel.lastPerception;
    newPOI.pointOnField = Transformation::robotToField(theRobotPose, newPOI.pointOnField);
    newPOI.waitForReached = false;
    newPOI.relativeAngle = theBallModel.lastPerception.angle();
    timeStampTarget = theFrameInfo.time;
    targetReached = false;
    targetQueue.push_back(newPOI);
    return;
  }
  
  if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 3000 && theBallModel.estimate.position.norm() < 1000)
  {
    headControlState = ballSweepWide;
    sweepField(-60_deg, 60_deg, 300, false, headSpeedOpt, false);
    return;
  }

  resetQueue();
  PointOfInterest newPOI;
  newPOI.maxSpeed = headSpeedMax;
  newPOI.timeAtTarget = 0;
  newPOI.pointOnField = Vector2f((theBallModel.estimate.position.x() + theBallModel.estimate.velocity.x()/2),
    (theBallModel.estimate.position.y() + theBallModel.estimate.velocity.y()/2));
  newPOI.pointOnField = Transformation::robotToField(theRobotPose,newPOI.pointOnField);
  newPOI.waitForReached = false;
  newPOI.relativeAngle = theBallModel.estimate.position.angle();
  timeStampTarget = theFrameInfo.time;
  targetReached = false;
  targetQueue.push_back(newPOI);
}

void HeadControl2014::calcHeadAngles()
{
  const float maxHeadPan = toDegrees(theJointCalibration.joints[Joints::headYaw].maxAngle);
  const float minHeadPan = toDegrees(theJointCalibration.joints[Joints::headYaw].minAngle);
  const float maxHeadTilt = toDegrees(theJointCalibration.joints[Joints::headPitch].maxAngle);
  const float minHeadTilt = toDegrees(theJointCalibration.joints[Joints::headPitch].minAngle);

  //basically from head symbols
  float headPan = 0.f;
  float headTilt = 0.f;

  if (theHeadControlRequest.controlType == HeadControlRequest::direct)
  {
    headPan = toDegrees(theHeadControlRequest.pan);
    headTilt = toDegrees(theHeadControlRequest.tilt);
  }
  else if (isLookingDown)
  {
    headPan = 0.f;
    headTilt = 30.f;
  }
  else
  {
    if (targetQueue.empty())
      return;

    const float distanceToTarget = (lastPOIRel-Transformation::fieldToRobot(theRobotPose,targetQueue.back().pointOnField)).norm();
    const float distanceToTargetAngle = toDegrees(std::abs(Angle::normalize(Transformation::fieldToRobot(theRobotPose,lastPOI).angle()-Transformation::fieldToRobot(theRobotPose,targetQueue.back().pointOnField).angle())));
    //const float distanceDifference = std::abs(lastPOI.norm()-targetQueue.back().pointOnField.norm());

    if (distanceToTarget < 50.f && distanceToTargetAngle < ((lastPOIRel.norm() < 500) ? 30 : 5))
    {
      return;
    }
    else
      lastPOI = targetQueue.back().pointOnField;

    Vector2f targetRel = Transformation::fieldToRobot(theRobotPose, lastPOI);
    lastPOIRel = targetRel;
    //Calculate the head-pan:
    headPan = toDegrees(std::atan2(targetRel.y(), targetRel.x()));
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
    Vector2f panTilt(headPan, 30_deg);
    Vector3f target(targetRel.x(), targetRel.y(), 0);
    // TODO: why is bh method not really working as expected (centering object)?
    const CameraMatrix &cm = trackWithUpper ? theCameraMatrixUpper : theCameraMatrix;
    float tiltAngleToTarget = std::atan2(cm.translation.z(), target.norm());
    //InverseKinematic::calcHeadJoints(target, pi_2-pi_8, theRobotDimensions, !trackWithUpper, panTilt, theCameraCalibration);
    //headTilt = toDegrees(panTilt.y());
    targetIsClose = (targetDistance < (targetIsClose ? 2500 : 2000));
    if (theBehaviorData.role == BehaviorData::keeper && targetIsClose)
      headTilt = 28;
    else
      headTilt = toDegrees(tiltAngleToTarget - theRobotDimensions.getTiltNeckToCamera(!trackWithUpper));
  }

  // clip angles
  Range<float> rangePan(minHeadPan,maxHeadPan);
  headPan = rangePan.limit(headPan);

  Range<float> rangeTilt(minHeadTilt,maxHeadTilt);
  headTilt = rangeTilt.limit(headTilt);

  // no minimal movement
  if (std::abs(toDegrees(theJointSensorData.angles[Joints::headYaw])-headPan) < 2.5)
    headPan = toDegrees(theJointSensorData.angles[Joints::headYaw]);

  newPan = Angle::fromDegrees(headPan);
  newTilt = Angle::fromDegrees(headTilt);
}

void HeadControl2014::calculatePointsOfInterest()
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
    if (theRobotPose.validity > 0.5f && theRobotPose.translation.x() > -1500 && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 1000)
      lookAtBall(false);
    else
    {
      headControlState = set;
      sweepField(-pi_4, pi_4, theRobotPose.translation.norm(), true, headSpeedOpt, false);
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

void HeadControl2014::handleReadyState()
{
  headControlState = ready;
  sweepField(-pi_4, pi_4, 3000, true, headSpeedOpt, false, false);
}

void HeadControl2014::handlePlayingState()
{
  if (theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 8000)
    lookAtBall();
  else
    sweepField(-90_deg, 90_deg, 300, true, headSpeedOpt);
}

void HeadControl2014::lookDown()
{
  isLookingDown = true;
}

void HeadControl2014::moveHead()
{
  const bool direct = 
    (theHeadControlRequest.controlType == HeadControlRequest::direct || targetQueue.empty());

  // from headmotionengine
  Angle lastPan = theJointSensorData.angles[Joints::headYaw];
  if (lastPan == JointAngles::off)
    lastPan = newPan;
  Angle lastTilt = theJointSensorData.angles[Joints::headPitch];
  if (lastTilt == JointAngles::off)
    lastTilt = newTilt;
  const Vector2f lastPosition(lastPan, lastTilt);
  const Vector2f target(((newPan == JointAngles::off) ? lastPan : newPan),
    ((newTilt == JointAngles::off) ? lastTilt : newTilt));
  const float distanceToTarget = (lastPosition-target).norm();

  localHeadAngleRequest.pan = newPan;
  localHeadAngleRequest.tilt = newTilt;
  localHeadAngleRequest.speed = direct ? headSpeedMax : targetQueue.back().maxSpeed;
  
  if (!direct)
  {
    float relativeAngleDiff = std::abs(targetQueue.back().relativeAngle - lastPan);
    relativeAngleReached = !relativeAngleReached && sgn(targetQueue.back().relativeAngle - lastPan) != sgn(newPan - lastPan);

    // TODO: check overshoot and reached stuff
    if ((distanceToTarget < 5_deg || 
      (relativeAngleDiff > targetQueue.back().maxOverShoot && relativeAngleReached) ||
      (targetQueue.back().waitForReached == false && theFrameInfo.getTimeSince(timeStampTarget) > timeForHeadMovement) || 
      theFrameInfo.getTimeSince(timeStampTarget) > 5000 ||
      (std::abs(headPanDistances.sum()) < 0.3f && theFrameInfo.getTimeSince(timeStampTarget) > 1000))
        && !targetReached)
    {
      timeStampTarget = theFrameInfo.time;
      targetReached = true;
    }
    if (targetReached && 
      ((theFrameInfo.getTimeSince(timeStampTarget) > targetQueue.back().timeAtTarget) || targetQueue.back().waitForReached == false))
    {
      targetQueue.pop_back();
      targetReached = false;
      relativeAngleReached = false;
    }
  }
}

void HeadControl2014::sweepField(const Angle leftAngle,
  const Angle rightAngle,
  const float distance,
  const bool forceSweepType,
  const Angle speed,
  bool sweepNearestFirst,
  bool waitForReached)
{
  if (targetQueue.empty()
    || (lastControlType != theHeadControlRequest.controlType && !forceSweepType)
    || headControlState != lastHeadControlState)
    //|| robotState != lastRobotState) TODO: check this
  {
    lastHeadControlState = headControlState;
    resetQueue();
    bool isLeft = theJointSensorData.angles[Joints::headYaw] > 0;
    if (!sweepNearestFirst)
      isLeft = leftAngle > rightAngle;
    PointOfInterest newPOI, newPOI2;
    newPOI.maxSpeed = speed;
    newPOI.pointOnField = Vector2f(
      isLeft ? distance*std::cos(leftAngle) : distance*std::cos(rightAngle),
      isLeft ? distance*std::sin(leftAngle) : distance*std::sin(rightAngle));
    newPOI.pointOnField.x() = std::abs(newPOI.pointOnField.x());
    newPOI.relativeAngle = newPOI.pointOnField.angle();
    newPOI.pointOnField = Transformation::robotToField(theRobotPose, newPOI.pointOnField);
    newPOI.timeAtTarget = 100;
    newPOI.waitForReached = waitForReached;
    newPOI.maxOverShoot = (waitForReached ? 40_deg : 15_deg);
    newPOI2.maxSpeed = speed;
    newPOI2.pointOnField = Vector2f(
      isLeft ? distance*std::cos(rightAngle) : distance*std::cos(leftAngle),
      isLeft ? distance*std::sin(rightAngle) : distance*std::sin(leftAngle));
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

MAKE_MODULE(HeadControl2014, behaviorControl)
