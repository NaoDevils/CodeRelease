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

    atBall = (theBehaviorData.soccerState == BehaviorData::controlBall && (theBallSymbols.ballPositionRelativeWOPreview.norm() < (atBall ? 400 : 300)));
    
    if (theRoleSymbols.role == BehaviorData::keeper && theGameInfo.state == STATE_PLAYING 
      && !(theGameSymbols.timeSinceLastPenalty < 15000)
      && !(theBehaviorConfiguration.noWLAN || !theTeammateData.wlanOK))
    {
      state = target;
    }
    else
    {
      switch (state)
      {
      case target:
        // if target position gets far away, switch to that
        // TODO: if target is behind me with similar rotation, stay
        if (distanceToTarget > (targetStateSwitchDistance + targetStateSwitchDistanceHysteresis) &&
          (!atBall) &&
          (theBehaviorData.soccerState == BehaviorData::controlBall || destRel.translation.x() > 0 || std::abs(destRel.rotation) > walkBackWardsRotation + 0.3f || distanceToTarget > walkBackWardsDistance + 300))
          state = far;
        break;
      case omni:
        if (distanceToTarget < targetStateSwitchDistance || atBall)
          state = target;
        else if (thePath.nearestObstacle > omniStateSwitchDistance + omniStateSwitchDistanceHysteresis ||
          std::abs(angleNextWayPointRelative) < omniStateSwitchAngle / 2)
          state = far;
        break;
      case far:
        if (distanceToTarget < targetStateSwitchDistance || atBall ||
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
    }

    // translation along path is reduced to theoretical maximum (actually not correct)
    // TODO: use speed sum stuff from request translator?
    Vector2f translationOnPath = thePath.wayPoints[1].translation - thePath.wayPoints[0].translation;
    translationOnPath.rotate((float)-theRobotPoseAfterPreview.rotation);

    float maxSpeedX = theWalkingEngineParams.speedLimits.xForward;
    if (state != far)
    {
      maxSpeedX = theWalkingEngineParams.speedLimits.xForwardOmni;
    }

    // if distance exceeds maxSpeeds, clip
    if (translationOnPath.x() > maxSpeedX || std::abs(translationOnPath.y()) > theWalkingEngineParams.speedLimits.y)
    {
      float xFactor = std::abs(translationOnPath.x()) / maxSpeedX;
      float yFactor = std::abs(translationOnPath.y()) / theWalkingEngineParams.speedLimits.y;
      translationOnPath /= std::max(0.001f, std::max(xFactor, yFactor));
    }
    
    inDribbling = false;
    float maxSpeedR = theWalkingEngineParams.speedLimits.r;
    if (state != far)
    {
      maxSpeedR = theWalkingEngineParams.speedLimits.r*0.8f;
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
      if (theBallSymbols.ballPositionRelative.norm() < 600 && std::abs(rotDiff) > pi3_4)
        rotDiff = theBallSymbols.ballPositionRelative.angle();
      /*float angleToTargetRel =
        Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);*/
      speedRequest.rotation = (float)(sgn(rotDiff)*std::min<float>(std::abs(rotDiff), theWalkingEngineParams.speedLimits.r));
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
      speedRequest.translation.x() = theWalkingEngineParams.speedLimits.xForward * (float)alpha;
      speedRequest.translation.y() = 0;
      alpha = std::min<float>(nextWPDistance / 500, 1);
      speedRequest.rotation = 
        (float)(angleNextWayPointRelative * alpha + Angle::normalize(thePath.wayPoints[1].rotation - theRobotPoseAfterPreview.rotation) * (1 - alpha));
      break;
    }
    default:
      break;
    } // end of switch of states
    if (useDistanceBasedSpeedPercentageInReady && theGameInfo.state == STATE_READY)
    {
      // if distance on path greater than 4.5 m for 45000 ms (at start of ready), go full speed
      // else decrease, but min speed is half max speed
      float speedPercentage = std::min(std::max(minSpeedPercentageInReady, std::max(thePath.length*10,7500.f) / std::max(45000 - theGameSymbols.timeSinceGameState, 1000)), 1.f);
      speedRequest.translation *= speedPercentage;
      speedRequest.rotation *= speedPercentage;
    }
    // ball chaser and goalie always at full speed, others only if near to ball
    else if (theBehaviorData.soccerState != BehaviorData::controlBall &&
      theBallSymbols.ballPositionRelative.norm() > 2000 &&
      theBehaviorData.role != BehaviorData::keeper)
    {
      speedRequest.translation *= speedPercentageWhenNotChasingBall;
      speedRequest.rotation *= speedPercentageWhenNotChasingBall;
    }

    // factors 4,2,2 are the speeds we typically do in one step for x/y/r
    // TODO: put all of this in PG
    
    speedRequest.translation.x() *= 4;
    
    speedRequest.translation.y() *= 2;

    speedRequest.rotation *= 2.f;

  } // end of if motionrequest is walk
}

MAKE_MODULE(PathToSpeedStable, pathPlanning)