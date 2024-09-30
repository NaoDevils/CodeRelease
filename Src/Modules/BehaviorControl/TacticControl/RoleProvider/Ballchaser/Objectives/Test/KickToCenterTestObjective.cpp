#include "KickToCenterTestObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>


KickToCenterTestObjective::KickToCenterTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName) : Objective("TestObjective", role, logger)
{
  std::vector<std::string> kickNames = {};
  kickNames.emplace_back(kickName);
  kicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), kickNames);
}

bool KickToCenterTestObjective::perform(Ballchaser& ballchaser)
{
  Kick* kick = KickUtils::unpack(kicks).at(0);
  const bool kickWithLeft = true;
  const Vector2f absoluteBallPosition = role->theBallSymbols.ballPositionFieldPredicted;

  Vector2f targetDirectionVector = Vector2f(0.f, 0.f) - role->theRobotPoseAfterPreview.translation;
  if (targetDirectionVector.x() == 0.f && targetDirectionVector.y() == 0.f)
  {
    targetDirectionVector = Vector2f(1.f, 0.f);
  }
  const Pose2f kickPose = kick->getKickPose(absoluteBallPosition, targetDirectionVector.angle(), kickWithLeft);
  const Vector2f target = absoluteBallPosition + targetDirectionVector.normalize(1000.f);
  currentKickManager.setCurrentKick(ballchaser, kick, kickPose, kickWithLeft, target, role->theBallSymbols, role->theFrameInfo);
  return true;
}

bool KickToCenterTestObjective::leaveCondition() const
{
  return false;
}

void KickToCenterTestObjective::postprocess()
{
  Objective::postprocess();
}
