#include "KickInPositionTestObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>

KickInPositionTestObjective::KickInPositionTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName) : Objective("TestObjective", role, logger)
{
  std::vector<std::string> kickNames = {};
  kickNames.emplace_back(kickName);
  kicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), kickNames);
}

bool KickInPositionTestObjective::perform(Ballchaser& ballchaser)
{
  if (KickUtils::isBallKicked(role->theMotionInfo))
  {
    kicked = true;
    return true;
  }
  const Vector2f littleOffCurrentPosition = {role->theRobotPoseAfterPreview.translation.x() + 1, role->theRobotPoseAfterPreview.translation.y()};
  const Pose2f littleOffCurrentPose = Pose2f(role->theRobotPoseAfterPreview.rotation, littleOffCurrentPosition);
  if (kicked)
  {
    ballchaser.optPosition = littleOffCurrentPose;
    kicked = false;
  }
  else
  {
    const Vector2f target = {0, 0};
    currentKickManager.setCurrentKick(ballchaser, kicks.at(0).get(), role->theRobotPoseAfterPreview, true, target, role->theBallSymbols, role->theFrameInfo);
  }
  return true;
}
