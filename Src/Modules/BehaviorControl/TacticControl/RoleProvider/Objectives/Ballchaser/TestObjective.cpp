#include "TestObjective.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysterUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuter45.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuterFoot.h>

TestObjective::TestObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("TestObjective", role, logger)
{
  //  kicks.push_back(std::make_unique<SlowLongKick>());
  //  kicks.push_back(std::make_unique<FastLongKick>());

  //  kicks.push_back(std::make_unique<KickHack>());
  //  kicks.push_back(std::make_unique<KickHackLong>());
  kicks.push_back(std::make_unique<Rotate45Kick>());
  //  kicks.push_back(std::make_unique<SideKickOuter45>());
  //kicks.push_back(std::make_unique<SideKickOuterFoot>());
}

bool TestObjective::perform(Ballchaser& ballchaser)
{
  kickManager.update(role->ballPosition, role->theRobotPoseAfterPreview, role->theWalkingEngineParams);

  /*
  PositionUtils::setPosition(ballchaser, {200, 0});
  PositionUtils::turnToPosition(ballchaser, {0, 0});
  return true;
  */

  /*
  const Vector2f targetPosition = Vector2f(0, 0);
  std::vector<ExecutableKick> executableKicks = kickManager.getExecutableKicks(
      role->theRobotPoseAfterPreview,
      role->ballPosition,
      targetPosition,
      DistanceRequirement::mayShorterOrFurther,
      KickUtils::unpack(kicks),
      role->theFieldDimensions,
      role->theHeatMapCollection,
      role->theRobotMap);
  if (executableKicks.empty())
  {
    return false;
  }
  kickManager.kickTo(
      ballchaser,
      role->theRobotPoseAfterPreview,
      role->ballPosition,
      executableKicks.at(0).target,
      *executableKicks.at(0).kick);
  return true;
  */

  ExecutableKicks executableKicks =
      kickManager
          .getExecutableKicks(role->theRobotPoseAfterPreview, role->ballPosition, KickUtils::unpack(kicks), role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap)
          .filterOutside(role->theFieldDimensions)
          .filterBlocked(role->theFieldDimensions, role->theRobotMap)
          .reduceToBest(-1, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, role->kickWithLeft, role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap);
  if (executableKicks.hasBest())
  {
    kickManager.kickTo(ballchaser, executableKicks);
  }
  return true;
}

bool TestObjective::leaveCondition() const
{
  return false;
}
