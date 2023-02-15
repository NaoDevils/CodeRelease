#pragma once

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/Optimize.h>
#include <optional>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Representations/Modeling/RobotMap.h"

class KickUtils
{

public:
  static Pose2f getKickPose(const Angle& robotRotation, const Vector2f& ballPosition, bool kickWithLeft, float afterRotation_optDistanceToBallX, float afterRotation_optDistanceToBallY);

  static Angle getFastestReachableKickAngleBetweenTargets(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static float getKickTime(const Kick& kick, const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, bool hysteresis, bool leftFootClosestToBall);

  static bool fulfillsDistanceRequirements(const Kick& kick, float targetDistance, DistanceRequirement distanceRequirement, bool hysteresis);

  static float getMinRobotToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const RobotMap& theRobotMap);

  static std::tuple<Vector2f, Vector2f> getLeftAndRightTarget(const Vector2f& position, const Vector2f& target1, const Vector2f& target2);

  static bool isKickToOutside(const Kick* kick, const Vector2f& ballPosition, const Vector2f& direction, bool hysteresis, const FieldDimensions& theFieldDimensions);

  static std::vector<Vector2f> getTargetsWithEvenDistance(const Vector2f& target1, const Vector2f& target2, int stepSize);

  static std::vector<Vector2f> getTargetsWithEvenAngle(const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static std::optional<Vector2f> getStraightAheadTarget(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static float getMinKickToObstaclesDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions, const RobotMap& theRobotMap);

  static float getMinKickToGoalPostDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions);

  static float getMinFieldBorderToKickDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const FieldDimensions& theFieldDimensions);

  static Pose2f getKickPose(const Vector2f& ballPosition, const Vector2f& targetPosition, bool rotateLeft, bool ballLeftFoot, Angle optAngle, float afterRotation_optDistanceToBallX, float afterRotation_optDistanceToBallY);

  static std::vector<Kick*> unpack(const std::vector<std::unique_ptr<Kick>>& kicks);

  static void drawKickRange(const Vector2f& ballPosition, const Geometry::Line& line, const int minAhead, const int maxAhead)
  {
    const Vector2f& target1 = line.base + line.direction * minAhead;
    const Vector2f& target2 = line.base + line.direction * maxAhead;
    drawKickInfos(ballPosition, target1, target2);
  }

  static void drawKickInfos(const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2)
  {
    LINE("behavior:BallchaserProvider:KickManager:KickRange", ballPosition.x(), ballPosition.y(), target1.x(), target1.y(), 20, Drawings::solidPen, ColorRGBA::blue);
    LINE("behavior:BallchaserProvider:KickManager:KickRange", ballPosition.x(), ballPosition.y(), target2.x(), target2.y(), 20, Drawings::solidPen, ColorRGBA::blue);
  }

private:
  static float getMinDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const std::vector<Vector2f>& obstacles);
};
