#pragma once

#include "FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include <optional>

class KickUtils
{

public:
  static Angle getFastestReachableKickAngleBetweenTargets(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static float getKickTime(const Kick& kick, const Pose2f& playerPose, const Vector2f& ballPosition, const Pose2f& kickPose);

  static bool fulfillsDistanceRequirements(const Kick& kick, float targetDistance, DistanceRequirement distanceRequirement, bool hysteresis);

  static float getMinRobotToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const RobotMap& theRobotMap);

  static std::tuple<Vector2f, Vector2f> getLeftAndRightTarget(const Vector2f& position, const Vector2f& target1, const Vector2f& target2);

  static std::vector<Vector2f> getTargetsWithEvenDistance(const Vector2f& target1, const Vector2f& target2, int stepSize);

  static std::vector<Vector2f> getTargetsWithEvenAngle(const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static std::optional<Vector2f> getStraightAheadTarget(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static float getMinKickToObstaclesDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  static float getMinKickToGoalPostDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions);

  static float getMinFieldBorderToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions);

  static bool isLeftOfKickLine(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& targetPosition, bool currentIsLeftOfLine);

  static Pose2f getKickPose(const Angle& robotRotation, const Vector2f& ballPosition, bool mirror, float afterRotation_optDistanceToBallX, float afterRotation_optDistanceToBallY);

  static Pose2f getKickPose(const Vector2f& ballPosition, const Vector2f& targetPosition, bool rotateLeft, bool mirror, Angle optAngle, float afterRotation_optDistanceToBallX, float afterRotation_optDistanceToBallY);

  static std::vector<Kick*> unpack(const std::vector<std::unique_ptr<Kick>>& kicks);
  static std::vector<Kick*> unpack(const std::vector<std::unique_ptr<Kick>>& kicks1, const std::vector<std::unique_ptr<Kick>>& kicks2);

  static bool isBallTouched(const Vector2f& relativeBallPosition, const RobotModel& theRobotModel, const FieldDimensions& theFieldDimensions);
  static bool isBallKicked(const MotionInfo& theMotionInfo);

private:
  static float getMinDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const std::vector<Vector2f>& obstacles);
};
