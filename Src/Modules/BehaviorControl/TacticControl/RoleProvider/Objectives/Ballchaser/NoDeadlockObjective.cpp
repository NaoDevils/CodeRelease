#include "NoDeadlockObjective.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BlockUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallchaserUtils.h>

NoDeadlockObjective::NoDeadlockObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("NoDeadlockObjective", role, logger) {}

bool NoDeadlockObjective::perform(Ballchaser& ballchaser)
{
  const Vector2f defensivePosition = BallchaserUtils::getWaitPosition(role->ballPosition, role->theFieldDimensions);
  PositionUtils::setPosition(ballchaser, defensivePosition);
  return true;
}
