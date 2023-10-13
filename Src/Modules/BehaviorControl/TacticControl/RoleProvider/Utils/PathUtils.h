#pragma once

#include "Tools/Math/Angle.h"
#include "MathUtils.h"
#include "Tools/Math/Geometry.h"

class PathUtils
{

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
    const float distanceTime = getDistanceTime(currentPose.translation, targetPose.translation, ballPosition, aroundBallFactor);

    /*
    const float t = turnTime + distanceTime;
    std::ostringstream s;
    s << turnTime << ", " << distanceTime << ", " << t;
    std::string var = s.str();
    OUTPUT_TEXT(var);
    */

    return turnTime + distanceTime;
  }

private:
  static float getTurnTime(const Pose2f& currentPose, const Pose2f& targetPose, const float aroundBallFactor)
  {
    const Angle TURN_ANGLE_PER_SECOND = 25_deg; // TODO Constants
    const float MIN_TURN_TO_WALK_DISTANCE = 300.f;
    const float MAX_TURN_TO_WALK_DISTANCE = 500.f;

    const float maxTurnTime = 180_deg / TURN_ANGLE_PER_SECOND;
    if (aroundBallFactor > 0.99f)
    {
      return maxTurnTime;
    }

    const float turnMultiplier = MathUtils::clamp_f(
        (Geometry::distance(currentPose.translation, targetPose.translation) - MIN_TURN_TO_WALK_DISTANCE) / (MAX_TURN_TO_WALK_DISTANCE - MIN_TURN_TO_WALK_DISTANCE), 0.f, 1.f);

    // Turn and walk. Don't calculate the time it takes to turn to walk because it's almost the same for every pose
    const Angle walkAngle = (targetPose.translation - currentPose.translation).angle();
    const Angle fromWalk_toTarget_angleDiff = MathUtils::getLargerMinusSmallerAngleDiff(walkAngle, targetPose.rotation);
    const float fromWalk_toTarget_time = fromWalk_toTarget_angleDiff / TURN_ANGLE_PER_SECOND;

    // Close enough to just walk (no need to turn since its so close the robot will walk sideways or backward)
    const Angle fromCurrent_toTarget_angleDiff = MathUtils::getLargerMinusSmallerAngleDiff(currentPose.rotation, targetPose.rotation);
    const float fromCurrent_toTarget_time = fromCurrent_toTarget_angleDiff / TURN_ANGLE_PER_SECOND;

    float time = turnMultiplier * fromWalk_toTarget_time + (1 - turnMultiplier) * fromCurrent_toTarget_time;

    return (1 - aroundBallFactor) * time + aroundBallFactor * maxTurnTime;
  }

  static float getDistanceTime(const Vector2f& playerCurrentPosition, const Vector2f& playerTargetPosition, const Vector2f& ballPosition, const float aroundBallFactor)
  {
    const float MAX_AIR_DISTANCE = 500.f;
    const float MAX_CURVE_DISTANCE = 200.f;
    const float WALK_MM_PER_SECOND = 150.f; // TODO Constant

    const float airDistance = Geometry::distance(playerCurrentPosition, playerTargetPosition);
    const float clampedAirDistance = std::min(airDistance, MAX_AIR_DISTANCE);
    const float clampedCurvedDistance = aroundBallFactor * MAX_CURVE_DISTANCE;
    return (clampedAirDistance + clampedCurvedDistance) / WALK_MM_PER_SECOND;
  }
};