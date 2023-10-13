#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/CurrentKick.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Selectables/KickPlan.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Math/Eigen.h"
#include "optional"
#include "stdexcept"

/**
 * 1. Remembers last kick decision
 * 2. Provides methods for complicated kick operations
 */
class KickManager
{

public:
  KickManager() = default;
  ~KickManager() = default;

  [[nodiscard]] bool isActive() const;
  void stop();
  std::optional<CurrentKick> getCurrentKick(const Vector2f& ballPosition);
  void kickTo(PositioningAndKickSymbols& pakSymbols, const KickPlan& kickPlan, const FrameInfo& theFrameInfo);

private:
  // Save current Kick to decide if hysteresis should be applied
  CurrentKick currentKick = {};
  unsigned currentKickTime = 0;
  Vector2f currentKickBallPosition = {};
};
