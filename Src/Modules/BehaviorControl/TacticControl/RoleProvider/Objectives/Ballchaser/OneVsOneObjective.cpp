#include "OneVsOneObjective.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysterUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHack.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Rotate45Kick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuter45.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuterFoot.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/BallchaserUtils.h>

OneVsOneObjective::OneVsOneObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("OneVsOneObjective", role, logger)
{
  kicks.push_back(std::make_unique<KickHack>());
  kicks.push_back(std::make_unique<KickHackLong>());
  kicks.push_back(std::make_unique<Rotate45Kick>());
  kicks.push_back(std::make_unique<SideKickOuter45>());
  //kicks.push_back(std::make_unique<SideKickOuterFoot>());
}

bool OneVsOneObjective::enterCondition()
{
  if (role->danger == Danger::HIGH)
  {
    logger.addSuccessReason("HighDanger");
    return true;
  }
  else
  {
    logger.addFailedReason("NotHighDanger");
    return false;
  }
}

void OneVsOneObjective::preprocess()
{
  Objective::preprocess();
}

bool OneVsOneObjective::perform(Ballchaser& ballchaser)
{
  kickManager.update(role->ballPosition, role->theRobotPoseAfterPreview, role->theWalkingEngineParams);

  if (role->playAgainstDribbleTeam)
  {
    logger.addNote("JumpToDefendIfAgainstDribblingTeam");
  }
  else
  {
    if (kickBallInCone(ballchaser, kickManager.isActive() ? 40_deg : 30_deg))
    {
      logger.addSuccessReason("KickInSmallerCone");
      return true;
    }
    else
    {
      logger.addFailedReason("KickInSmallerCone");
    }
  }

  updateTimer();

  if (!startTimeSet)
  {
    defend(ballchaser);
    logger.addNote("TimerNotStarted");
    logger.addSuccessReason("WalkToBall");
    return true;
  }

  if (!isTimeout())
  {
    defend(ballchaser);
    logger.addSuccessReason("Defend");
    return true;
  }

  if (kickBallInCone(ballchaser, kickManager.isActive() ? 60_deg : 50_deg))
  {
    logger.addSuccessReason("KickInBiggerCone");
    return true;
  }
  else
  {
    defend(ballchaser);
    logger.addFailedReason("KickInBiggerCone");
    logger.addSuccessReason("Defend");
    return true;
  }
}

void OneVsOneObjective::updateTimer()
{
  if (startTimeSet)
  {
    if (!isInBallArea(startTimeBallPosition))
    {
      logger.addNote("ballMoved->stopTimer");
      startTimeSet = false;
      return;
    }
    if (!isInBallArea(role->theRobotPoseAfterPreview.translation))
    {
      logger.addNote("playerMoved->stopTimer");
      startTimeSet = false;
      return;
    }
    logger.addNote("TimerRunning");
    startTimeBallPosition = role->ballPosition;
    return;
  }
  logger.addNote("TimerNotRunning");
  if (isInBallArea(role->theRobotPoseAfterPreview.translation))
  {
    logger.addNote("playerCloseToBall->startTimer");
    startTime = role->theFrameInfo.time;
    startTimeSet = true;
    startTimeBallPosition = role->ballPosition;
  }
}

bool OneVsOneObjective::isInBallArea(const Vector2f& position)
{
  Geometry::Circle closeToBallCircle = Geometry::Circle(role->ballPosition, 500.f); // TODO Constant
  return Geometry::isPointInCircle(closeToBallCircle, position);
}

bool OneVsOneObjective::isTimeout()
{
  const bool timeOut = role->theFrameInfo.getTimeSince(startTime) > 5000;
  if (timeOut)
  {
    logger.addNote("Timeout");
    return true;
  }
  else
  {
    logger.addNote("NoTimeout");
    return false;
  }
}

bool OneVsOneObjective::kickBallInCone(Ballchaser& ballchaser, const Angle toPlayerMaxAngle)
{
  const Angle TO_OPPONENT_SIDE_MAX_ANGLE = 95_deg;
  const Angle MIN_ANGLE_SIZE = 10_deg;

  playerIsLeftOfBall = HysterUtils::comparePossibleNegatives(playerIsLeftOfBall, role->ballPosition.y(), role->theRobotPoseAfterPreview.translation.y(), 200.f); // TODO Constant

  const Range range = Range(-TO_OPPONENT_SIDE_MAX_ANGLE, TO_OPPONENT_SIDE_MAX_ANGLE);
  const Angle currentAngle = (role->ballPosition - role->theRobotPoseAfterPreview.translation).angle();

  Angle angle1 = currentAngle + toPlayerMaxAngle;
  angle1 = range.limit(angle1);

  Angle angle2 = currentAngle - toPlayerMaxAngle;
  angle2 = range.limit(angle2);
  angle2 = std::min(Angle(angle1 - MIN_ANGLE_SIZE), angle2);

  const KickRange kickRange = {role->ballPosition, angle1, angle2, 1000.f, DistanceRequirement::mayShorterOrFurther};

  ExecutableKicks executableKicks =
      kickManager
          .getExecutableKicks(role->theRobotPoseAfterPreview, kickRange, 10, KickUtils::unpack(kicks), role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap)
          .filterOutside(role->theFieldDimensions)
          .filterBlocked(role->theFieldDimensions, role->theRobotMap)
          .reduceToBest(-1.f, 0.f, 0.f, 0.f, 0.1f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, role->kickWithLeft, role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap);

  if (executableKicks.hasBest())
  {
    kickManager.kickTo(ballchaser, executableKicks);
    return true;
  }
  return false;
}

bool OneVsOneObjective::justKick(Ballchaser& ballchaser)
{
  ExecutableKicks executableKicks =
      kickManager
          .getExecutableKicks(role->theRobotPoseAfterPreview, role->ballPosition, KickUtils::unpack(kicks), role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap)
          .filterOutside(role->theFieldDimensions)
          .reduceToBest(-1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, role->kickWithLeft, role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap);

  const bool suc = executableKicks.hasBest();
  if (suc)
  {
    kickManager.kickTo(ballchaser, executableKicks);
  }
  return suc;
}

void OneVsOneObjective::defend(Ballchaser& ballchaser)
{
  PositionUtils::setPosition(ballchaser, BallchaserUtils::getWaitPosition(role->ballPosition, role->theFieldDimensions));
  PositionUtils::turnToPosition(ballchaser, role->ballPosition);
}

void OneVsOneObjective::postprocess()
{
  Objective::postprocess();
  startTimeSet = false;
}
