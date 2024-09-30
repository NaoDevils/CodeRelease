#pragma once

#include "FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include <optional>
#include <memory>

class KickUtils
{

public:
  static Angle getFastestReachableKickAngleBetweenTargets(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static std::tuple<Vector2f, Vector2f> getLeftAndRightTarget(const Vector2f& position, const Vector2f& target1, const Vector2f& target2);

  static std::vector<Vector2f> getTargetsWithEvenDistance(const Vector2f& target1, const Vector2f& target2, int stepSize);

  static std::vector<Vector2f> getTargetsWithEvenAngle(const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static std::optional<Vector2f> getStraightAheadTarget(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& target1, const Vector2f& target2);

  static std::vector<Kick*> unpack(const std::vector<std::unique_ptr<Kick>>& kicks);
  static std::vector<Kick*> unpack(const std::vector<std::unique_ptr<Kick>>& kicks1, const std::vector<std::unique_ptr<Kick>>& kicks2);

  static bool isBallTouched(const Vector2f& relativeBallPosition, const RobotModel& theRobotModel, const FieldDimensions& theFieldDimensions);
  static bool isBallKicked(const MotionInfo& theMotionInfo);

private:
  static float getMinDistance(const Vector2f& ballPosition, const Vector2f& targetPosition, const std::vector<Vector2f>& obstacles);
};
