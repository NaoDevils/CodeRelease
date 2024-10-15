#include "CurrentKickManager.h"

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Eigen.h"

std::optional<CurrentKick> CurrentKickManager::getCurrentKick(const Vector2f& ballPosition)
{
  ASSERT(std::isnormal(ballPosition.x()) || ballPosition.x() == 0.0f);
  ASSERT(std::isnormal(ballPosition.y()) || ballPosition.y() == 0.0f);

  if (!hasCurrentKick())
  {
    return {};
  }

  const float STOP_KICK_IF_BALL_MOVED_DISTANCE = 200.f;

  const Vector2f ballMoved = (ballPosition - currentKickBallPosition);
  if (ballMoved.norm() > STOP_KICK_IF_BALL_MOVED_DISTANCE)
  {
    ANNOTATION("KickManager", "BallMoved! Stop " << currentKick.kick->name);
    deleteCurrentKick();
    return {};
  }

  return currentKick;
}

void CurrentKickManager::deleteCurrentKick()
{
  if (hasCurrentKick())
  {
    //ANNOTATION("KickManager", "Stopped " << currentKick->getName());
    currentKick = {};
    currentKickTime = 0;
    currentKickBallPosition = {};
  }
}

void CurrentKickManager::setCurrentKick(PositioningAndKickSymbols& pakSymbols, const ExecutableShot& executableShot, const FrameInfo& theFrameInfo)
{
  const SelectablePose& selectablePose = executableShot.selectablePose;
  setCurrentKick(pakSymbols,
      selectablePose.getBallPosition(),
      selectablePose.selectableShot.selectableTarget.selectableKick.kick,
      selectablePose.pose,
      selectablePose.kickWithLeft,
      selectablePose.selectableShot.selectableTarget.target,
      theFrameInfo);
}

void CurrentKickManager::setCurrentKick(PositioningAndKickSymbols& pakSymbols, const SimpleExecutableShot& simpleExecutableShot, const FrameInfo& theFrameInfo)
{
  setCurrentKick(pakSymbols, simpleExecutableShot.ballPosition, simpleExecutableShot.kick, simpleExecutableShot.kickPose, simpleExecutableShot.kickWithLeft, simpleExecutableShot.target, theFrameInfo);
}

void CurrentKickManager::setCurrentKick(
    PositioningAndKickSymbols& pakSymbols, const Vector2f& ballPosition, Kick* kick, const Pose2f& kickPose, const bool kickWithLeft, const Vector2f& target, const FrameInfo& theFrameInfo)
{
  const unsigned REFRESH_CURRENT_KICK_AFTER_DURATION = 2000; // milliseconds

  // Debug Drawings and Annotations
  pakSymbols.log_kickName = kick->name;

  // Set values
  kick->perform(pakSymbols, kickPose, kick->kickWithLeftToMirror(kickWithLeft), target); // TODO remove variable start-

  // Save new values
  currentKick = {target, kick, kickWithLeft};
  if (!hasCurrentKick() || theFrameInfo.time - currentKickTime > REFRESH_CURRENT_KICK_AFTER_DURATION)
  {
    currentKickTime = theFrameInfo.time;
    currentKickBallPosition = ballPosition;
  }
}

bool CurrentKickManager::hasCurrentKick() const
{
  return currentKick.kick != nullptr;
}
