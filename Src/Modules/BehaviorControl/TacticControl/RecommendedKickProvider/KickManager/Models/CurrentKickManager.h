#pragma once

#include <Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/SimpleExecutableShot.h>
#include "CurrentKick.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Selectables/ExecutableShot.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Math/Eigen.h"
#include "optional"
#include "stdexcept"

class CurrentKickManager
{

public:
  CurrentKickManager() = default;
  ~CurrentKickManager() = default;

  void deleteCurrentKick();
  [[nodiscard]] std::optional<CurrentKick> getCurrentKick(const Vector2f& ballPosition);
  void setCurrentKick(PositioningAndKickSymbols& pakSymbols, const ExecutableShot& executableShot, const FrameInfo& theFrameInfo);
  void setCurrentKick(PositioningAndKickSymbols& pakSymbols, const SimpleExecutableShot& simpleExecutableShot, const FrameInfo& theFrameInfo);
  void setCurrentKick(PositioningAndKickSymbols& pakSymbols, const Vector2f& ballPosition, Kick* kick, const Pose2f& kickPose, const bool kickWithLeft, const Vector2f& target, const FrameInfo& theFrameInfo);

private:
  /**
   * Even if this returns true the method getCurrentKick is allowed to delete the currentKick and return an empty optional!
   */
  [[nodiscard]] bool hasCurrentKick() const;

  // Save current Kick to decide if hysteresis should be applied
  CurrentKick currentKick = {};
  unsigned currentKickTime = 0;
  Vector2f currentKickBallPosition = {};
};
