#include "KickManager.h"

#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Constants.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Eigen.h"

bool KickManager::isActive() const
{
  return currentKick.kick != nullptr;
}

std::optional<CurrentKick> KickManager::getCurrentKick(const Vector2f& ballPosition)
{
  ASSERT(std::isnormal(ballPosition.x()) || ballPosition.x() == 0.0f);
  ASSERT(std::isnormal(ballPosition.y()) || ballPosition.y() == 0.0f);

  if (!isActive())
  {
    return {};
  }

  const float STOP_KICK_IF_BALL_MOVED_DISTANCE = 200.f;

  const Vector2f ballMoved = (ballPosition - currentKickBallPosition);
  if (ballMoved.norm() > STOP_KICK_IF_BALL_MOVED_DISTANCE)
  {
    ANNOTATION("KickManager", "BallMoved! Stop " << currentKick.kick->getName());
    stop();
    return {};
  }

  return currentKick;
}

void KickManager::stop()
{
  if (isActive())
  {
    //ANNOTATION("KickManager", "Stopped " << currentKick->getName());
    currentKick = {};
    currentKickTime = 0;
    currentKickBallPosition = {};
  }
}

void KickManager::kickTo(PositioningAndKickSymbols& pakSymbols, const KickPlan& kickPlan, const FrameInfo& theFrameInfo)
{
  const unsigned REFRESH_CURRENT_KICK_AFTER_DURATION = 2000; // milliseconds

  const SelectablePose& selectablePose = kickPlan.selectablePose;
  const Vector2f& target = selectablePose.selectableTarget.target;
  Kick* kick = selectablePose.selectableTarget.selectableKick.kick;

  // Debug Drawings and Annotations
  CIRCLE(DRAW_EXECUTABLE_KICK_TARGET_AREA, target.x(), target.y(), kick->getInaccuracy(), 12, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
  pakSymbols.log_kickName = kick->getName();

  // Set values
  kick->perform(pakSymbols, selectablePose.pose, kick->kickWithLeftToMirror(selectablePose.kickWithLeft), target,
      true); // TODO remove variable start-

  // Save new values
  currentKick = {target, kick, selectablePose.kickWithLeft};
  if (!isActive() || theFrameInfo.time - currentKickTime > REFRESH_CURRENT_KICK_AFTER_DURATION)
  {
    currentKickTime = theFrameInfo.time;
    currentKickBallPosition = selectablePose.selectableTarget.ballPosition;
  }
}
