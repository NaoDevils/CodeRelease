#include "PathToSpeedStable.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>
#include <functional>

PathToSpeedStable::PathToSpeedStable()
{
  state = far;

  inOwnGoalArea = false;
}

void PathToSpeedStable::update(SpeedRequest& speedRequest)
{
  // TODO: ball next to goal post
  
  speedRequest.translation = Vector2f::Zero();
  speedRequest.rotation = 0.f;

  // TODO : remember the old way
  if (theMotionRequest.motion == MotionRequest::walk)
  {
    if (theMotionRequest.walkRequest.requestType != WalkRequest::destination)
    {
      speedRequest.translation = theMotionRequest.walkRequest.request.translation;
      speedRequest.rotation = theMotionRequest.walkRequest.request.rotation;
      return;
    }
    Pose2f destWorldCoordinates = Pose2f(theRobotPoseAfterPreview + theMotionRequest.walkRequest.request);
    // TODO: acc
    if (thePath.wayPoints.size() < 2)
      return;

    /**** decide state ****/
    Pose2f destRel = destWorldCoordinates - theRobotPoseAfterPreview;
    float distanceToTarget = destRel.translation.norm();
    float angleNextWayPointRelative =
      Angle::normalize((thePath.wayPoints[1].translation - thePath.wayPoints[0].translation).angle() - theRobotPoseAfterPreview.rotation);
    switch (state)
    {
    case target:
      // if target position gets far away, switch to that
      // TODO: if target is behind me with similar rotation, stay
      if (distanceToTarget > (targetStateSwitchDistance + targetStateSwitchDistanceHysteresis) &&
        (theBehaviorData.soccerState == BehaviorData::controlBall || destRel.translation.x() > 0 || std::abs(destRel.rotation) > walkBackWardsRotation + 0.3f || distanceToTarget > walkBackWardsDistance + 300))
        state = far;
      break;
    case omni:
      if (distanceToTarget < targetStateSwitchDistance)
        state = target;
      else if (thePath.nearestObstacle > omniStateSwitchDistance + omniStateSwitchDistanceHysteresis ||
        std::abs(angleNextWayPointRelative) < omniStateSwitchAngle / 2)
        state = far;
      break;
    case far:
      if (distanceToTarget < targetStateSwitchDistance || 
        (theBehaviorData.soccerState != BehaviorData::controlBall && destRel.translation.x() < 0 && distanceToTarget < walkBackWardsDistance && std::abs(destRel.rotation) < walkBackWardsRotation))
        //&& distanceToTarget < 800))
        state = target;
      else if (thePath.nearestObstacle < omniStateSwitchDistance &&
        std::abs(angleNextWayPointRelative) > omniStateSwitchAngle)
        state = omni;
      break;
    default:
      // should not happen..
      break;
    }

    // translation along path is reduced to theoretical maximum (actually not correct)
    // TODO: use speed sum stuff from request translator?
    Vector2f translationOnPath = thePath.wayPoints[1].translation - thePath.wayPoints[0].translation;
    translationOnPath.rotate((float)-theRobotPoseAfterPreview.rotation);
    float maxAddedSpeedInTheory =
      std::sqrt((float)theWalkingEngineParams.maxSpeedY*theWalkingEngineParams.maxSpeedY +
      theWalkingEngineParams.maxSpeedXForward*theWalkingEngineParams.maxSpeedXForward);

    translationOnPath.normalize(std::min<float>(maxAddedSpeedInTheory, translationOnPath.norm()));

    float yFactor = std::max<float>(1, std::abs(translationOnPath.y()) / theWalkingEngineParams.maxSpeedY);
    translationOnPath /= yFactor;
    // HACK for faster approach
    if (std::abs(translationOnPath.y()) < 30 && translationOnPath.x() > 0 && translationOnPath.x() < 100)
    {
      translationOnPath.x() = std::min(translationOnPath.x()*2.f, 100.f); // TODO: different for different kick types
      translationOnPath.y() *= 2.5f;
    }

    switch (state)
    {
    case target:
    {
      // turn towards target rotation, keep target in sight, TODO!
      // TODO: do not use closest angle, but angle to keep target in sight
      const float targetRot = thePath.wayPoints.back().rotation;
      float rotDiff = Angle::normalize(targetRot - theRobotPoseAfterPreview.rotation);
      // need angle of pose to target position to check if target is in sight
      if (theBallModel.estimate.position.norm() < 600 && std::abs(rotDiff) > pi3_4)
        rotDiff = theBallModel.estimate.position.angle();
      /*float angleToTargetRel =
        Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);*/
      speedRequest.rotation = (float)(sgn(rotDiff)*std::min<float>(std::abs(rotDiff), theWalkingEngineParams.maxSpeedR));
      /*if (distanceToTarget > 250)
      {
      double nextAngleToTargetRel = normalize(angleToTargetRel + speedRequest.rotation);
      if (std::abs(nextAngleToTargetRel) > 0.8) // TODO: param
      {
      speedRequest.rotation = angleToTargetRel;
      }
      }*/
      speedRequest.translation = translationOnPath;
      break;
    }
    case omni:
    {
      // no rotation!!!?!?!?!? 
      speedRequest.rotation = 0;
      speedRequest.translation = translationOnPath;

      break;
    }
    case far:
    {
      // in this state, only rotation and x speeds are used.
      // nearly only rotation when rotation too high
      // TODO: test & make sure only rotation is used if value is too high
      // TODO: values to params
      float alpha = 1.f;
      float nextWPDistance = (thePath.wayPoints[1].translation - thePath.wayPoints[0].translation).norm();
      if (std::abs(angleNextWayPointRelative) > pi_4)
      {
        alpha = std::max<float>(0.f, 1.f - (std::abs(angleNextWayPointRelative) - pi_4) / pi_4);
        alpha /= std::max<float>(1.f, 1500 / (nextWPDistance + 1));
      }
      else if (thePath.nearestObstacle < 750 && std::abs(angleNextWayPointRelative) > Angle::fromDegrees(30))
        alpha = std::min<float>(alpha, 0.5f);
      speedRequest.translation.x() = theWalkingEngineParams.maxSpeedXForward * (float)alpha;
      speedRequest.translation.y() = 0;
      alpha = std::min<float>(nextWPDistance / 500, 1);
      speedRequest.rotation = 
        (float)(angleNextWayPointRelative * alpha + Angle::normalize(thePath.wayPoints[1].rotation - theRobotPoseAfterPreview.rotation) * (1 - alpha));
      break;
    }
    default:
      break;
    }
  }
}

MAKE_MODULE(PathToSpeedStable, pathPlanning)