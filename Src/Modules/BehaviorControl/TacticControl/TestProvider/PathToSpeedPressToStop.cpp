#include "PathToSpeedPressToStop.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <functional>

PathToSpeedPressToStop::PathToSpeedPressToStop()
{
  InMapFile stream("simplePathProvider.cfg");
  if (stream.exists())
  {
    stream >> pathParameters;
  }
  else
  {
    pathParameters.robotInfluenceRadius = 300.f;
    pathParameters.teamRobotInfluenceRadius = 400.f;
    pathParameters.centerCircleInfluenceRadius = 350.f;
    pathParameters.setPlayInfluenceRadius = 800.f;
  }
}

void PathToSpeedPressToStop::update(SpeedRequest& speedRequest)
{
  handleStopRequest();
  if (stopRequestedByButton)
  {
    speedRequest.rotation = 0_deg;
    speedRequest.translation = Vector2f::Zero();
    return;
  }
  // somehow walk towards ball (no specified kind of step requested)
  // Ingmar, 15.07.2020: Does not work like this, reimplement if needed
  if (theMotionRequest.motion == MotionRequest::walk && theMotionRequest.walkRequest.requestType == WalkRequest::destination
      && theMotionRequest.walkRequest.rotationType == WalkRequest::towardsBall && theMotionRequest.walkRequest.stepRequest == WalkRequest::none)
  {
    speedRequest.rotation = theBallSymbols.ballPositionRelative.angle();
    speedRequest.translation = Transformation::fieldToRobot(theRobotPoseAfterPreview, thePath.wayPoints[1].translation);
    return;
  }

  // TODO: ball next to goal post

  speedRequest.translation = Vector2f::Zero();
  speedRequest.rotation = 0.f;

  // dont move if nearly falling
  if (theSensorControlParams.sensorControlActivation.activateSpeedReduction
      && std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y())
          >= theSensorControlParams.speedReduction.emergencyStopSpeedReductionFactorThreshold)
  {
    return;
  }

  // TODO : remember the old way


  if (theMotionRequest.motion == MotionRequest::walk)
  {
    // walk towards destination
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
    float angleNextWayPointRelative = Angle::normalize((thePath.wayPoints[1].translation - thePath.wayPoints[0].translation).angle() - theRobotPoseAfterPreview.rotation);

    atBall = (theBehaviorData.soccerState == BehaviorData::controlBall && (theBallSymbols.ballPositionRelativeWOPreview.norm() < (atBall ? 600 : 450)) && theBallSymbols.timeSinceLastSeen < 2000);

    //Todo: Test
    //Goalie should only be in state target if game state is playing
    bool goalieCloseToGoal = theRobotPoseAfterPreview.translation.x() < (theFieldDimensions.xPosOwnPenaltyArea + 500); // only in goal area

    // TODO: seems to be a behavior thing to decide this
    // keeper in playing, not chasing the ball (not after penalty, not without wlan)
    if (theRoleSymbols.role == BehaviorData::keeper && theGameInfo.state == STATE_PLAYING && goalieCloseToGoal && theGameSymbols.timeSinceLastPenalty >= 15000 && theTeammateData.commEnabled)
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
        if (distanceToTarget > (targetStateSwitchDistance + targetStateSwitchDistanceHysteresis) && (!atBall)
            && (theBehaviorData.soccerState == BehaviorData::controlBall || destRel.translation.x() > 0 || std::abs(destRel.rotation) > walkBackWardsRotation + 0.3f
                || distanceToTarget > walkBackWardsDistance + 300))
          state = far;
        break;
      case omni:
        if (distanceToTarget < targetStateSwitchDistance || atBall)
          state = target;
        else if (thePath.nearestObstacle > omniStateSwitchDistance + omniStateSwitchDistanceHysteresis || std::abs(angleNextWayPointRelative) < omniStateSwitchAngle / 2)
          state = far;
        break;
      case far:
        if (distanceToTarget < targetStateSwitchDistance || atBall
            || (theBehaviorData.soccerState != BehaviorData::controlBall && destRel.translation.x() < 0 && distanceToTarget < walkBackWardsDistance && std::abs(destRel.rotation) < walkBackWardsRotation))
          //&& distanceToTarget < 800))
          state = target;
        else if (thePath.nearestObstacle < omniStateSwitchDistance && std::abs(angleNextWayPointRelative) > omniStateSwitchAngle)
          state = omni;
        break;
      default:
        // should not happen..
        break;
      }
    }

    // translation along path is reduced to theoretical maximum (actually not correct)
    Vector2f translationOnPath = thePath.wayPoints[1].translation - thePath.wayPoints[0].translation;
    translationOnPath.rotate(-theRobotPoseAfterPreview.rotation);

    float maxSpeedX = theWalkingEngineParams.speedLimits.xForward * theWalkingEngineParams.speedFactor;
    if (state != far)
    {
      maxSpeedX = theWalkingEngineParams.speedLimits.xForwardOmni * theWalkingEngineParams.speedFactor;
    }

    // if distance exceeds maxSpeeds, clip
    if (translationOnPath.x() > maxSpeedX || std::abs(translationOnPath.y()) > (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor))
    {
      float xFactor = std::abs(translationOnPath.x()) / maxSpeedX;
      float yFactor = std::abs(translationOnPath.y()) / (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor);
      translationOnPath /= std::max(0.001f, std::max(xFactor, yFactor));
    }

    inDribbling = false;
    float maxSpeedR = theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor;
    if (state != far)
    {
      maxSpeedR = theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor * 0.8f;
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
      if (atBall && std::abs(rotDiff) > pi3_4)
        rotDiff = theBallSymbols.ballPositionRelative.angle();
      /*float angleToTargetRel =
        Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);*/
      speedRequest.rotation = (float)(sgn(rotDiff) * std::min<float>(std::abs(rotDiff), theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor));
      /*if (atBall && std::abs(theBallSymbols.ballPositionRelative.angle()) > pi3_4)
        speedRequest.translation = Vector2f::Zero();
      else*/
      {
        // at ball, do not rotate translation fully to speed up going around ball

        float rotationFactor = (atBall && theGameInfo.setPlay == SET_PLAY_NONE) ? walkAroundBallTranslationRotationFactor : 1.f;
        translationOnPath.rotate(-speedRequest.rotation * rotationFactor);
        speedRequest.translation = translationOnPath;
        if (std::abs(speedRequest.translation.y()) > 80 && std::abs(speedRequest.translation.x()) < 50 && speedRequest.translation.x() > -50.f)
          speedRequest.translation.x() = std::min(-50.f, speedRequest.translation.x());
      }
      break;
    }
    case omni:
    {
      Angle rotationOffsetToTarget = Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);
      speedRequest.rotation = (std::abs(rotationOffsetToTarget) > 20_deg) ? rotationOffsetToTarget : 0_deg;
      speedRequest.translation = (std::abs(speedRequest.rotation) > 60_deg) ? Vector2f::Zero() : translationOnPath;
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
      speedRequest.translation.x() = theWalkingEngineParams.speedLimits.xForward * theWalkingEngineParams.speedFactor * (float)alpha;
      speedRequest.translation.y() = 0;
      alpha = std::min<float>(nextWPDistance / 500, 1);
      speedRequest.rotation = (float)(angleNextWayPointRelative * alpha + Angle::normalize(thePath.wayPoints[1].rotation - theRobotPoseAfterPreview.rotation) * (1 - alpha));
      break;
    }
    default:
      break;
    } // end of switch of states

    // if we are close to obstacles, combine translation vector with a vector moving away from obstacle
    // only exception is at ball, since we fight there!
    // no clipping done here..
    if (!atBall)
    {
      float influenceRadius = influenceRadiusOfObstacleOnTranslation;
      float obstacleInfluence = influenceOfObstacleOnTranslation;
      switch (thePath.nearestObstacleType)
      {
      case Path::ball:
        influenceRadius = pathParameters.ballInfluenceRadius;
        break;
      case Path::robot:
        influenceRadius = pathParameters.robotInfluenceRadius;
        break;
      case Path::teamRobot:
        influenceRadius = pathParameters.teamRobotInfluenceRadius;
        obstacleInfluence = influenceOfObstacleOnTranslationTeammate;
        break;
      case Path::centerCircle:
        influenceRadius = pathParameters.centerCircleInfluenceRadius + theFieldDimensions.centerCircleRadius;
        obstacleInfluence = influenceOfObstacleOnTranslationCenterCircle;
        break;
      case Path::setPlayCircle:
        influenceRadius = pathParameters.setPlayInfluenceRadius;
        obstacleInfluence = influenceOfObstacleOnTranslationSetPlay;
        break;
      default:
        break;
      }
      float obstacleDistance = std::max(0.f, -thePath.nearestObstacle); // nearestObstacle is negative if we enter obstacle influence radius!
      float distanceInfluence = (influenceRadius - obstacleDistance);
      float obstacleTranslationInfluence = std::max(0.f, obstacleInfluence - (distanceInfluence * obstacleInfluence) / influenceRadius);
      Vector2f obstacleRelative = Transformation::fieldToRobot(theRobotPoseAfterPreview, thePath.nearestObstaclePosition);

      speedRequest.translation = speedRequest.translation * (1.f - obstacleTranslationInfluence) - obstacleRelative.normalize(200.f) * obstacleTranslationInfluence;
    }

    if (useDistanceBasedSpeedPercentageInReady && theGameInfo.state == STATE_READY)
    {
      // if distance on path greater than 4.5 m for 45000 ms (at start of ready), go full speed
      // else decrease, but min speed is half max speed
      float speedPercentage = std::min(std::max(minSpeedPercentageInReady, std::max(thePath.length * 10, 7500.f) / std::max(45000 - theGameSymbols.timeSinceGameState, 1000)), 1.f);
      speedRequest.translation *= speedPercentage;
      speedRequest.rotation *= speedPercentage;
    }
    // ball chaser and goalie always at full speed, others only if near to ball
    else if (theBehaviorData.soccerState != BehaviorData::controlBall && theBallSymbols.ballPositionRelative.norm() > 2000 && theBehaviorData.role != BehaviorData::keeper)
    {
      speedRequest.translation *= speedPercentageWhenNotChasingBall;
      speedRequest.rotation *= speedPercentageWhenNotChasingBall;
    }

    float rotationLimit = (speedRequest.translation.x() == 0.f && speedRequest.translation.y() == 0.f)
        ? static_cast<float>(theWalkingEngineParams.speedLimits.rOnly * theWalkingEngineParams.speedFactor)
        : maxSpeedR;

    // factors 4,2,2 are the speeds we typically do in one step for x/y/r
    // This is done to reach target within a single fast step since
    // speed = distance would result in four steps to reach target and thus
    // slowly approaching target
    // TODO: only PatterGenerator can really know what to do here
    float stepSpeedX = speedRequest.translation.x() * 4;
    float stepSpeedY = speedRequest.translation.y() * 2;
    float stepSpeedR = speedRequest.rotation * 2;
    if (stepSpeedX < maxSpeedX && stepSpeedX > (-theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedFactor)
        && stepSpeedY < (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor)
        && stepSpeedY > (-theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor) && std::abs(stepSpeedR) < rotationLimit)
    {
      speedRequest.translation.x() *= 4;

      speedRequest.translation.y() *= 2;

      speedRequest.rotation *= 2.f;
    }
    float factorX = (speedRequest.translation.x() > 0)
        ? speedRequest.translation.x() / maxSpeedX
        : speedRequest.translation.x() / (-theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedFactor);
    float factorY = std::abs(speedRequest.translation.y()) / (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor);
    float factorR = std::abs(speedRequest.rotation / rotationLimit);
    float maxFactor = std::max(factorX, std::max(factorY, factorR));
    if (maxFactor > 1.f)
    {
      speedRequest.translation /= maxFactor;
      speedRequest.rotation /= maxFactor;
    }

  } // end of if motionrequest is walk
}

/**
 * \brief Create a not moving speed request if a stop is requested.
 * 
 * Stops can only requested when live configuration is enabled (check LiveConfigurationState),
 * noConfiguration is the active configuration mode. A press of the front head button requests a 
 * stop. During the stop the speed request will be set so that the robot does not move.
 * The behavior will otherwise not be influenced. If the robot is already in a stop and another
 * stop request occurs the robot will start moving like normal again.
 * 
 * \param speedRequest The SpeedRequest to fill this frame.
 */
void PathToSpeedPressToStop::handleStopRequest()
{
  bool noConfig = theLiveConfigurationState.currentConfigurationState == LiveConfigurationState::ConfigurationState::noConfiguration;
  bool toggleStop = theLiveConfigurationState.liveConfigurationActive && noConfig && theLiveConfigurationState.headFrontPressedThisFrame;
  stopRequestedByButton = toggleStop ? !stopRequestedByButton : stopRequestedByButton;
}

MAKE_MODULE(PathToSpeedPressToStop, pathPlanning)
