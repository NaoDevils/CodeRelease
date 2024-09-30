#pragma once

#include "MathUtils.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Geometry.h"

class PathUtils
{

#define TURN_ANGLE_PER_SECOND 25_deg
#define MIN_TURN_TO_WALK_DISTANCE 700.f
#define MAX_TURN_TO_WALK_DISTANCE 1200.f
#define MAX_TURN_TIME (180_deg / TURN_ANGLE_PER_SECOND)

#define MAX_CURVE_DISTANCE 500.f
#define WALK_MM_PER_SECOND 150.f

public:
  static float getPathTime(const Pose2f& currentPose, const Pose2f& targetPose, const Vector2f& ballPosition)
  {
    const Angle IF_HIGHER_WALK_CURVE_ANGLE = 90_deg;
    const Angle MAX_WALK_CURVE_ANGLE = 180_deg;

    const Angle currentToBallAngle = Angle::normalize((ballPosition - currentPose.translation).angle());
    const Angle ballToTargetAngle = Angle::normalize((ballPosition - targetPose.translation).angle());
    const Angle aroundBallAngleDiff = MathUtils::getAngleSmallestDiff(currentToBallAngle, ballToTargetAngle);

    const float aroundBallFactor = MathUtils::clamp_f((aroundBallAngleDiff - IF_HIGHER_WALK_CURVE_ANGLE) / (MAX_WALK_CURVE_ANGLE - IF_HIGHER_WALK_CURVE_ANGLE), 0.f, 1.f);

    const float turnTime = getTurnTime(currentPose, targetPose, aroundBallFactor);
    const float distanceTime = getDistanceTime(currentPose.translation, targetPose.translation, aroundBallFactor);

    return turnTime + distanceTime;
  }

private:
  static float getTurnTime(const Pose2f& currentPose, const Pose2f& targetPose, const float aroundBallFactor)
  {
    // Assumes that the robot walks straight to the targetPose.translation. Calculates the time the robot needs to turn from the assumed walking angle to the targetPose.angle
    const Angle walkAngle = (targetPose.translation - currentPose.translation).angle();
    const Angle fromWalk_toTarget_angleDiff = MathUtils::getAngleSmallestDiff(walkAngle, targetPose.rotation);
    const float fromWalk_toTarget_time = fromWalk_toTarget_angleDiff / TURN_ANGLE_PER_SECOND;

    // Assumes that the robot will walk sideways or backwards to the targetPose.translation. Calculates the time the robot needs to rotate from the currentPose.angle to the targetPose.angle
    const Angle fromCurrent_toTarget_angleDiff = MathUtils::getAngleSmallestDiff(currentPose.rotation, targetPose.rotation);
    const float fromCurrent_toTarget_time = fromCurrent_toTarget_angleDiff / TURN_ANGLE_PER_SECOND;

    // Calculate a transition between the two calculated times
    const float walkDistance = Geometry::distance(currentPose.translation, targetPose.translation);
    const float transitionMultiplier = MathUtils::clamp_f((walkDistance - MIN_TURN_TO_WALK_DISTANCE) / (MAX_TURN_TO_WALK_DISTANCE - MIN_TURN_TO_WALK_DISTANCE), 0.f, 1.f);
    float time = transitionMultiplier * fromWalk_toTarget_time + (1 - transitionMultiplier) * fromCurrent_toTarget_time;

    return (1 - aroundBallFactor) * time + aroundBallFactor * MAX_TURN_TIME;
  }

  static float getDistanceTime(const Vector2f& playerCurrentPosition, const Vector2f& playerTargetPosition, const float aroundBallFactor)
  {
    const float airDistance = Geometry::distance(playerCurrentPosition, playerTargetPosition);
    const float clampedCurvedDistance = aroundBallFactor * MAX_CURVE_DISTANCE;
    return (airDistance + clampedCurvedDistance) / WALK_MM_PER_SECOND;
  }
};