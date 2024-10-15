#include "KickBallTestObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>

KickBallTestObjective::KickBallTestObjective(BallchaserProvider* role, BehaviorLogger& logger, const std::string& kickName) : Objective("TestObjective", role, logger)
{
  std::vector<std::string> kickNames = {};
  kickNames.emplace_back(kickName);
  kicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), kickNames);
}

bool KickBallTestObjective::perform(Ballchaser& ballchaser)
{
  Kick* kick = KickUtils::unpack(kicks).at(0);
  const bool kickWithLeft = true;
  const Vector2f ballPosition = role->theBallSymbols.ballPositionFieldPredicted;
  const Vector2f playerPosition = role->theRobotPoseAfterPreview.translation;

  const float playerToBallDistance = (ballPosition - playerPosition).norm();
  Vector2f targetDirectionVector = {};
  if (playerToBallDistance < 1000.f && targetDirectionVectorOptional.has_value())
  {
    targetDirectionVector = targetDirectionVectorOptional.value();
  }
  else
  {
    targetDirectionVector = ballPosition - playerPosition;
    if (targetDirectionVector.x() == 0.f && targetDirectionVector.y() == 0.f)
    {
      targetDirectionVector = Vector2f(1.f, 0.f);
    }
    targetDirectionVectorOptional = targetDirectionVector;
  }
  const Pose2f kickPose = kick->getKickPose(ballPosition, targetDirectionVector.angle(), kickWithLeft);
  const Vector2f target = ballPosition + targetDirectionVector.normalize(1000.f);
  currentKickManager.setCurrentKick(ballchaser, ballPosition, kick, kickPose, kickWithLeft, target, role->theFrameInfo);
  return true;
}
