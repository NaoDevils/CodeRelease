#pragma once

#include "optional"
#include "stdexcept"
#include "CurrentKickManager.h"
#include "Selectables/SimpleExecutableShot.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SimpleExecutableShot.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include <optional>

class KickManager
{

public:
  KickManager() = default;
  ~KickManager() = default;

  void deleteCurrentKick() { currentKickManager.deleteCurrentKick(); }

  bool kickTo(
      PositioningAndKickSymbols& role, const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, const bool kickToTargetElseFurther, std::vector<Kick*> kicks, const FrameInfo& theFrameInfo)
  {
    if (kicks.empty())
    {
      return false;
    }

    // unpack
    const float targetDirection = (targetPosition - ballPosition).angle();
    const float targetDistance = Geometry::distance(ballPosition, targetPosition);

    // filter
    if (!kickToTargetElseFurther)
    {
      std::vector<Kick*> furtherThanTargetKicks = {};
      for (Kick* kick : kicks)
      {
        if (kick->maxDistance < targetDistance)
        {
          furtherThanTargetKicks.push_back(kick);
        }
      }
      if (!furtherThanTargetKicks.empty())
      {
        kicks = furtherThanTargetKicks;
      }
    }

    // find best
    Kick* bestKick = nullptr;
    bool bestKickWithLeft = false;
    Pose2f bestKickPose = {};
    float bestKickTime = std::numeric_limits<float>::max();
    float bestKickDistanceDeviation = std::numeric_limits<float>::max();
    for (Kick* kick : kicks)
    {
      const float kickDistanceDeviation = getKickDistanceDeviation(targetDistance, kick);
      const bool kickHigherDeviation = kickDistanceDeviation > bestKickDistanceDeviation;
      if (bestKick != nullptr && kickToTargetElseFurther && kickHigherDeviation)
      {
        continue;
      }

      Pose2f leftKickPose = kick->getKickPose(ballPosition, targetDirection, true);
      Pose2f rightKickPose = kick->getKickPose(ballPosition, targetDirection, false);

      const float leftKickTime = kick->time + PathUtils::getPathTime(playerPose, leftKickPose, ballPosition);
      const float rightKickTime = kick->time + PathUtils::getPathTime(playerPose, rightKickPose, ballPosition);

      std::optional<CurrentKick> currentKick = currentKickManager.getCurrentKick(ballPosition);
      const bool hasCurrentKick = currentKick.has_value();
      const bool allowLeft = hasCurrentKick ? currentKick.value().kickWithLeft : true;
      const bool allowRight = hasCurrentKick ? !currentKick.value().kickWithLeft : true;

      const bool leftKickFaster = leftKickTime < bestKickTime;
      const bool rightKickFaster = rightKickTime < bestKickTime;

      if (bestKick == nullptr || (allowLeft && leftKickFaster))
      {
        bestKick = kick;
        bestKickWithLeft = true;
        bestKickPose = leftKickPose;
        bestKickTime = leftKickTime;
        bestKickDistanceDeviation = kickDistanceDeviation;
      }
      if (allowRight && rightKickFaster)
      {
        bestKick = kick;
        bestKickWithLeft = false;
        bestKickPose = rightKickPose;
        bestKickTime = rightKickTime;
        bestKickDistanceDeviation = kickDistanceDeviation;
      }
    }

    if (bestKick == nullptr)
    {
      return false;
    }

    const SimpleExecutableShot simpleExecutableShot = SimpleExecutableShot(ballPosition, bestKick, bestKickPose, bestKickWithLeft, targetPosition);
    currentKickManager.setCurrentKick(role, simpleExecutableShot, theFrameInfo);

    return true;
  }

private:
  CurrentKickManager currentKickManager = {};
  static float getKickDistanceDeviation(const float distance, const Kick* kick)
  {
    if (kick->minDistance < distance && distance < kick->maxDistance)
    {
      return 0.f;
    }
    if (kick->minDistance < distance)
    {
      return distance - kick->minDistance;
    }
    // distance < kick->maxDistance
    return 2 * (kick->maxDistance - distance);
  }
};
