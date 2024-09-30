#include "PathToSpeedStable.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>
#include <functional>

PathToSpeedStable::PathToSpeedStable()
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

  for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
  {
    gyroDataBuffersX[i].fill(0.f);
    gyroDataBuffersY[i].fill(0.f);
  }
}

void PathToSpeedStable::update(SpeedRequest& speedRequest)
{
  DECLARE_DEBUG_DRAWING("module:PathToSpeedStable:avoidBallIntermediateTarget", "drawingOnField");
  DECLARE_PLOT("module:PathToSpeedStable:gyroVariance");

  if (theSensorControlParams.sensorControlActivation.activateSpeedReduction)
  {
    if (!theFootpositions.inKick)
      for (int i = 0; i < JoinedIMUData::numOfInertialDataSources; i++)
      {
        gyroDataBuffersX[i].push_front(theJoinedIMUData.imuData[i].gyro.x());
        gyroDataBuffersY[i].push_front(theJoinedIMUData.imuData[i].gyro.y());
      }


    Angle gyroVariance = (gyroDataBuffersY[theSensorControlParams.speedReduction.anglesource].getVariance() + gyroDataBuffersX[theSensorControlParams.speedReduction.anglesource].getVariance());
    PLOT("module:PathToSpeedStable:gyroVariance", gyroVariance);
    float maxFallDownSpeedReductionFactor = std::max<float>(theMotionState.walkingStatus.fallDownSpeedReductionFactor.x(), theMotionState.walkingStatus.fallDownSpeedReductionFactor.y());
    PLOT("module:PathToSpeedStable:maxFallDownSpeedReductionFactor", maxFallDownSpeedReductionFactor);

    float gyroVarianceThreshold = wasInEmergencyStop ? (theSensorControlParams.speedReduction.emergencyStopGyroVariance * 0.15f) : theSensorControlParams.speedReduction.emergencyStopGyroVariance;
    float emergencyStopSpeedReductionFactorThreshold = wasInEmergencyStop ? (1.2f) : theSensorControlParams.speedReduction.emergencyStopSpeedReductionFactorThreshold;

    if (theFootpositions.inKick)
    {
      gyroVarianceThreshold *= 2.f;
      emergencyStopSpeedReductionFactorThreshold *= 2.f;
    }
    PLOT("module:PathToSpeedStable:emergencyStopGyroVariance", gyroVarianceThreshold);
    PLOT("module:PathToSpeedStable:emergencyStopSpeedReductionFactorThreshold", emergencyStopSpeedReductionFactorThreshold);

    // dont move if nearly falling
    if (theMotionRequest.motion == MotionRequest::walk && (maxFallDownSpeedReductionFactor >= emergencyStopSpeedReductionFactorThreshold || gyroVariance >= gyroVarianceThreshold))
    {
      speedRequest.translation = Vector2f::Zero();
      speedRequest.rotation = 0.f;
      wasInEmergencyStop = true;
      return;
    }
  }


  wasInEmergencyStop = false;
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
    // TODO: acc
    if (thePath.wayPoints.size() < 2)
      return;

    /**** decide state ****/
    Pose2f destWorldCoordinates = Pose2f(theRobotPoseAfterPreview + theMotionRequest.walkRequest.request);
    Pose2f destRel = destWorldCoordinates - theRobotPoseAfterPreview;
    float distanceToTarget = destRel.translation.norm();
    float angleNextWayPointRelative = Angle::normalize((thePath.wayPoints[1].translation - thePath.wayPoints[0].translation).angle() - theRobotPoseAfterPreview.rotation);

    atBall = (theBehaviorData.soccerState == BehaviorData::controlBall && (theBallSymbols.ballPositionRelativeWOPreview.norm() < (atBall ? 600 : 450)) && theBallSymbols.timeSinceLastSeen < 2000);

    //Todo: Test
    //Goalie should only be in state target if game state is playing
    bool goalieCloseToGoal = theRobotPoseAfterPreview.translation.x() < (theFieldDimensions.xPosOwnPenaltyArea + 500); // only in goal area

    // TODO: seems to be a behavior thing to decide this
    // keeper in playing, not chasing the ball (not after penalty, not without wlan)
    if (theRoleSymbols.role == BehaviorData::keeper && theGameInfo.state == STATE_PLAYING && goalieCloseToGoal && theGameSymbols.timeSinceLastPenalty >= 15000
        && theTeammateData.commEnabled && keeperInPenaltyAreaTargetOnly)
    {
      state = target;
    }
    else
    {
      const float targetStateSwitchDistance = theRoleSymbols.role == BehaviorData::RoleAssignment::keeper ? targetStateSwitchDistanceKeeper : targetStateSwitchDistanceFieldplayer;
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
    float maxSpeedR = theWalkingEngineParams.speedLimits.rOnly * theWalkingEngineParams.speedFactor;
    if (state != far)
    {
      maxSpeedR = theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor;
    }

    switch (state)
    {
    case target:
    {
      // turn towards target rotation, keep target in sight, TODO!
      // TODO: do not use closest angle, but angle to keep target in sight
      const float targetRot = thePath.wayPoints.back().rotation;
      Vector2f vecRobotToBall = theBallSymbols.ballPositionField - theRobotPoseAfterPreview.translation;
      Vector2f vecTargetToBall = theBallSymbols.ballPositionField - thePath.wayPoints.back().translation;
      float rotDiff = Angle::normalize(vecTargetToBall.angle() - vecRobotToBall.angle());
      // need angle of pose to target position to check if target is in sight
      rotateAroundBall = atBall && std::abs(rotDiff) > (rotateAroundBall ? pi_4 : (pi_4 + pi_8));
      if (rotateAroundBall)
        rotDiff = theBallSymbols.ballPositionRelative.angle() * 1.5f;
      /*float angleToTargetRel =
        Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);*/
      speedRequest.rotation = (float)(sgn(rotDiff) * std::min<float>(std::abs(rotDiff), theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor));
      /*if (atBall && std::abs(theBallSymbols.ballPositionRelative.angle()) > pi3_4)
        speedRequest.translation = Vector2f::Zero();
      else*/
      if (rotateAroundBall)
      {
        // at ball, translate along increasingly smaller circle around ball
        const float circleRadius = std::min(std::max(walkAroundBallMinDistance, static_cast<float>(toDegrees(rotDiff) * 30)), walkAroundBallMaxDistance);
        Vector2f ballToRobot = theRobotPoseAfterPreview.translation - theBallSymbols.ballPositionField;
        ballToRobot.normalize(circleRadius);
        const Vector2f ballToTarget = thePath.wayPoints.back().translation - theBallSymbols.ballPositionField;
        Angle targetToRobotSeenFromBall = Angle::normalize(ballToRobot.angle() - ballToTarget.angle());
        bool avoidLeftAsSeenFromBall = targetToRobotSeenFromBall < 0_deg;
        ballToRobot.rotate(30_deg * (avoidLeftAsSeenFromBall ? 1.f : -1.f));
        Vector2f intermediateTargetWC = theBallSymbols.ballPositionField + ballToRobot;
        CROSS("module:PathToSpeedStable:avoidBallIntermediateTarget", intermediateTargetWC.x(), intermediateTargetWC.y(), 20, 3, Drawings::solidPen, ColorRGBA::yellow);
        speedRequest.translation = intermediateTargetWC - theRobotPoseAfterPreview.translation;
        speedRequest.translation.rotate(-theRobotPoseAfterPreview.rotation); // to relative coords
      }
      else
      {
        rotDiff = Angle::normalize(targetRot - theRobotPoseAfterPreview.rotation);
        speedRequest.translation = translationOnPath;
        speedRequest.rotation = (float)(sgn(rotDiff) * std::min<float>(std::abs(rotDiff), theWalkingEngineParams.speedLimits.r * theWalkingEngineParams.speedFactor));
      }
      break;
    }
    case omni:
    {
      Angle rotationOffsetToTarget = Angle::normalize((thePath.wayPoints.back().translation - theRobotPoseAfterPreview.translation).angle() - theRobotPoseAfterPreview.rotation);
      speedRequest.rotation = (std::abs(rotationOffsetToTarget) > 15_deg) ? rotationOffsetToTarget : 0_deg;
      speedRequest.translation = (std::abs(speedRequest.rotation) > 30_deg) ? Vector2f::Zero() : translationOnPath;
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
      case Path::ballchaser:
        influenceRadius = pathParameters.ballchaserInfluenceRadius;
        obstacleInfluence = influenceOfObstacleOnTranslationBallchaser;
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

    bool speedLimited = false;
    if (useDistanceBasedSpeedPercentageInReady && theGameInfo.state == STATE_READY)
    {
      // if distance on path greater than 4.5 m for 45000 ms (at start of ready), go full speed
      // else decrease, but min speed is half max speed
      float speedPercentage = std::min(std::max(minSpeedPercentageInReady, std::max(thePath.length * 10, 7500.f) / std::max(45000 - theGameSymbols.timeSinceGameState, 1000)), 1.f);
      speedRequest.translation *= speedPercentage;
      speedRequest.rotation *= speedPercentage;
      speedLimited = true;
    }
    // ball chaser and goalie always at full speed, others only if near to ball
    else if (theBehaviorData.soccerState != BehaviorData::controlBall && theBallSymbols.ballPositionRelative.norm() > 2000 && theBehaviorData.role != BehaviorData::keeper)
    {
      speedRequest.translation *= speedPercentageWhenNotChasingBall;
      speedRequest.rotation *= speedPercentageWhenNotChasingBall;
      speedLimited = true;
    }

    if (theBehaviorData.behaviorState >= BehaviorData::BehaviorState::firstCalibrationState)
    {
      speedRequest.translation *= speedPercentageCalibration;
      speedRequest.rotation *= speedPercentageCalibration;
      speedLimited = true;
    }

    float rotationLimit = (speedRequest.translation.x() == 0.f && speedRequest.translation.y() == 0.f)
        ? static_cast<float>(theWalkingEngineParams.speedLimits.rOnly * theWalkingEngineParams.speedFactor)
        : maxSpeedR;

    if (!speedLimited) // only do this if we want full speed approach, i.e. not in ready/camera calibration ..
    {
      // factors 4,2,2 are the speeds we typically do in one step for x/y/r
      // This is done to reach target within a single fast step since
      // speed = distance would result in four steps to reach target and thus
      // slowly approaching target
      // TODO: only PatterGenerator can really know what to do here
      float stepSpeedX = speedRequest.translation.x() * 5;
      float stepSpeedY = speedRequest.translation.y() * 3;
      float stepSpeedR = speedRequest.rotation * 3;
      if (stepSpeedX < maxSpeedX && stepSpeedX > (-theWalkingEngineParams.speedLimits.xBackward * theWalkingEngineParams.speedFactor)
          && stepSpeedY < (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor)
          && stepSpeedY > (-theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedFactor) && std::abs(stepSpeedR) < rotationLimit)
      {
        speedRequest.translation.x() = stepSpeedX;

        speedRequest.translation.y() = stepSpeedY;

        speedRequest.rotation = stepSpeedR;
      }
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

MAKE_MODULE(PathToSpeedStable, pathPlanning)
