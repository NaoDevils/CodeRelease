#pragma once

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Tools/Math/Eigen.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/ExecutableKicks/ExecutableKick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/ExecutableKicks/ExecutableKicks.h"
#include <stdexcept>
#include <optional>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/DangerUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h>
#include <Representations/MotionControl/WalkingEngineParams.h>

class KickManager
{

public:
  KickManager() = default;
  ~KickManager() = default;

  // Direct kick selection =============================================================================================

  bool kickInRangeCenter(Ballchaser& ballchaser,
      const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const Vector2f& target1,
      const Vector2f& target2,
      int stepSize,
      DistanceRequirement distanceRequirement,
      bool optimizeForTime,
      float minKickWidth,
      float minFreeAroundTarget,
      std::vector<Kick*> kicks,
      const FieldDimensions& theFieldDimensions,
      const RobotMap& theRobotMap);
  /**
   * Starts at the optimalAhead Position on the line. Tries left and right onside the line around this position until
   * a valid kick is found.
   * @return If a kick is found, kick to it and return true, else return false.
   */
  bool kickCloseToOptimalPosition(Ballchaser& ballchaser,
      const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const Geometry::Line& line,
      int minAhead,
      int optimalAhead,
      int maxAhead,
      int stepSize,
      DistanceRequirement distanceRequirement,
      bool optimizeForTime,
      float minKickWidth,
      float minFreeAroundTarget,
      std::vector<Kick*> kicks,
      const FieldDimensions& theFieldDimensions,
      const RobotMap& theRobotMap);

  std::tuple<Kick*, Vector2f> getBestKick(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const Vector2f& targetPosition,
      DistanceRequirement distanceRequirement,
      bool optimizeForTime,
      float minKickWidth,
      float minFreeAroundTarget,
      std::vector<Kick*>& kicks,
      const FieldDimensions& theFieldDimensions,
      const RobotMap& theRobotMap);

  // Generate object to select kick ====================================================================================

  ExecutableKicks getExecutableKicks(const Pose2f& playerPose,
      const KickRange& kickRange,
      int stepSizeOrZeroForAngle,
      std::vector<Kick*> kicks,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap);

  std::vector<ExecutableKick> getExecutableKicks(const Pose2f& playerPose,
      const Vector2f& ballPosition,
      const Vector2f& targetPosition,
      DistanceRequirement distanceRequirement,
      std::vector<Kick*> kicks,
      const FieldDimensions& theFieldDimensions,
      const HeatMapCollection& theHeatMapCollection,
      const RobotMap& theRobotMap);

  ExecutableKicks getExecutableKicks(
      const Pose2f& playerPose, const Vector2f& ballPosition, std::vector<Kick*> kicks, const FieldDimensions& theFieldDimensions, const HeatMapCollection& theHeatMapCollection, const RobotMap& theRobotMap);

  void kickTo(Ballchaser& ballchaser, const ExecutableKicks& executableKicks);

  // Interact with the kickManager =====================================================================================

  void stop();

  void update(const Vector2f& ballPosition, const RobotPoseAfterPreview& theRobotPoseAfterPreview, const WalkingEngineParams& theWalkingEngineParams);

  void kickTo(Ballchaser& ballchaser, const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, Kick& kick);

  bool isActive();

private:
  bool isCurrent(const Kick* kick, const Vector2f& kickTarget);

  Kick* currentKick = nullptr;
  Pose2f currentKickPose;
  Vector2f currentBallPosition;
  Vector2f currentKickTarget;
  bool currentKickStarted = false;

  bool leftFootClosestToBall = false;
};
