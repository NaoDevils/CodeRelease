#include "CurrentKickManager.h"

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/Constants.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Kick.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Eigen.h"

std::optional<CurrentKick> CurrentKickManager::getCurrentKick(const BallSymbols& theBallSymbols)
{
  if (!hasCurrentKick())
  {
    return {};
  }

  const float BALL_MOVED_ABSOLUTE_DISTANCE = 300.f;
  const Vector2f absoluteBallMovement = (theBallSymbols.ballPositionFieldPredicted - currentKickAbsoluteBallPosition);
  const bool ballMovedAbsolute = absoluteBallMovement.norm() > BALL_MOVED_ABSOLUTE_DISTANCE;

  const bool robotRotationChangedALot = false; // TODO

  if (ballMovedAbsolute || robotRotationChangedALot)
  {
    /*
     * localization problem
     *
     * If the robot is about to shoot an own goal but realized it in the last second, the hysteresis may not allow the robot to redecide.
     * If the kick is resettet the robot may redecide multiple times in situations where kicking fast is most important.
     * Setting the hysteresis to a solid but not extreme level allows to redecide in own goal situations but not in less important 1v1 situations on the field.
     */
    currentKick.hysteresis = Hysteresis::FORCE_HIGH;
    return currentKick;
  }

  currentKick.hysteresis = Hysteresis::YES;
  return currentKick;
}

void CurrentKickManager::deleteCurrentKick()
{
  if (hasCurrentKick())
  {
    //ANNOTATION("KickManager", "Stopped " << currentKick->getName());
    currentKick = {};
    currentKickAbsoluteBallPosition = {};
    currentKickRelativeBallPosition = {};
  }
}

void CurrentKickManager::setCurrentKick(PositioningAndKickSymbols& pakSymbols, const ExecutableShot& executableKick, const BallSymbols& theBallSymbols, const FrameInfo& theFrameInfo)
{
  const SelectablePose& selectablePose = executableKick.selectablePose;
  setCurrentKick(pakSymbols,
      selectablePose.selectableShot.selectableTarget.selectableKick.kick,
      selectablePose.pose,
      selectablePose.kickWithLeft,
      selectablePose.selectableShot.selectableTarget.target,
      theBallSymbols,
      theFrameInfo);
}

void CurrentKickManager::setCurrentKick(
    PositioningAndKickSymbols& pakSymbols, Kick* kick, const Pose2f& kickPose, const bool kickWithLeft, const Vector2f& target, const BallSymbols& theBallSymbols, const FrameInfo& theFrameInfo)
{
  // Debug Drawings and Annotations
  pakSymbols.log_kickName = kick->name;

  // Set values
  kick->perform(pakSymbols, kickPose, kick->kickWithLeftToMirror(kickWithLeft), target);

  // Save new values
  currentKick = {target, kick, kickWithLeft};
  currentKickAbsoluteBallPosition = theBallSymbols.ballPositionFieldPredicted;
  currentKickRelativeBallPosition = theBallSymbols.ballPositionRelativePredicted;
}

bool CurrentKickManager::hasCurrentKick() const
{
  return currentKick.kick != nullptr;
}
