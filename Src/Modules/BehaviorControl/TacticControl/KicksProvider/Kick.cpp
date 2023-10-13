#include "Kick.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickWithLeftUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/SelectedFoot.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"

bool Kick::getKickWithLeft(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, const SelectedFoot currentSelectedFoot) const
{
  switch (kickWithLeftCondition)
  {
  case KickInfos::KickWithLeftCondition::onLeftSide:
    return KickWithLeftUtils::getKickWithLeftIfOnLeftSide(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  case KickInfos::KickWithLeftCondition::onRightSide:
    return KickWithLeftUtils::getKickWithLeftIfOnRightSide(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  case KickInfos::KickWithLeftCondition::leftFootIsClosest:
    return KickWithLeftUtils::getKickWithLeftIfLeftFootIsClosest(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  case KickInfos::KickWithLeftCondition::rightFootIsClosest:
    return KickWithLeftUtils::getKickWithLeftIfRightFootIsClosest(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  default:
    throw std::exception();
  }
}

bool Kick::kickWithLeftToMirror(bool kickWithLeft) const
{
  if (switchKickFoot)
  {
    return kickWithLeft;
  }
  else
  {
    return !kickWithLeft;
  }
}

bool Kick::getKickWithLeftToTurnLeft(bool kickWithLeft) const
{
  if (switchKickFoot)
  {
    return kickWithLeft;
  }
  else
  {
    return !kickWithLeft;
  }
}

Pose2f Kick::getKickPose(const Vector2f& ballPosition, const Vector2f& targetPosition, bool kickWithLeft) const
{
  const bool turnLeft = getKickWithLeftToTurnLeft(kickWithLeft);

  const Angle ballToTargetAngle = (targetPosition - ballPosition).angle();
  const Angle optFieldAngleUnNormalized = ballToTargetAngle + (turnLeft ? 1.f : -1.f) * optAngle;
  const Angle optFieldAngle = Angle::normalize(optFieldAngleUnNormalized);

  const bool mirror = kickWithLeftToMirror(kickWithLeft);

  return KickUtils::getKickPose(optFieldAngle, ballPosition, mirror, optXDistanceToBall, optYDistanceToBall);
}