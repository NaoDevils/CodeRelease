#include "Kick.h"

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

Pose2f Kick::getKickPose(const Vector2f& ballPosition, const Angle targetDirection, bool kickWithLeft) const
{
  const bool turnLeft = getKickWithLeftToTurnLeft(kickWithLeft);

  const Angle optFieldAngleUnNormalized = targetDirection + (turnLeft ? 1.f : -1.f) * optAngle;
  const Angle optFieldAngle = Angle::normalize(optFieldAngleUnNormalized);

  const bool mirror = kickWithLeftToMirror(kickWithLeft);

  Pose2f pose(optFieldAngle, ballPosition);
  pose = pose.translate(-optXDistanceToBall, (mirror ? +1.f : -1.f) * optYDistanceToBall);
  return pose;
}