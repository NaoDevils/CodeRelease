#include "GoalObjective.h"

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysterUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/FastLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SlowLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHack.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHackVeryLong.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/Rotate45Kick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuter45.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SideKickOuterFoot.h>

GoalObjective::GoalObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("GoalObjective", role, logger), kickManager()
{
  //kicks.push_back(std::make_unique<SlowLongKick>());
  kicks.push_back(std::make_unique<KickHack>());
  kicks.push_back(std::make_unique<KickHackLong>());
  kicks.push_back(std::make_unique<KickHackVeryLong>());
  kicks.push_back(std::make_unique<Rotate45Kick>());
  kicks.push_back(std::make_unique<SideKickOuter45>());
  //kicks.push_back(std::make_unique<SideKickOuterFoot>());
}

bool GoalObjective::enterCondition()
{
  const Angle MAX_ANGLE = 74_deg; // TODO Constant also used in ExecutableKicks to filter
  const bool leftAngleTooSharp = HysterUtils::compareAbsolutes(openingAngleTooSharp, MAX_ANGLE, -role->theGoalSymbols.leftAngleBallToOppGoalWC, 2_deg);
  const bool rightAngleTooSharp = HysterUtils::compareAbsolutes(openingAngleTooSharp, MAX_ANGLE, role->theGoalSymbols.rightAngleBallToOppGoalWC, 2_deg);
  openingAngleTooSharp = leftAngleTooSharp || rightAngleTooSharp;
  if (openingAngleTooSharp)
  {
    logger.addFailedReason("openingAngleTooSharp");
    return false;
  }
  else
  {
    return true;
  }
}

bool GoalObjective::perform(Ballchaser& ballchaser)
{
  update();

  if (goalDistance == VERY_FAR)
  {
    logger.addFailedReason("TooFar");
    return false;
  }
  if (goalDistance == FAR) //  && role->danger != HIGH
  {
    return kickToGoalCenter(ballchaser);
  }
  return kickBestToGoal(ballchaser);
}

void GoalObjective::update()
{
  kickManager.update(role->ballPosition, role->theRobotPoseAfterPreview, role->theWalkingEngineParams);

  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal};
  const Geometry::Line goalPostLine = {leftGoalPostPosition, (rightGoalPostPosition - leftGoalPostPosition).normalized()};
  const float ballToGoalDistance = Geometry::getDistanceToLine(goalPostLine, role->ballPosition);

  GoalDistance newGoalDistance;

  if (isVeryCloseToGoal(leftGoalPostPosition, rightGoalPostPosition, ballToGoalDistance))
  {
    logger.addNote("VeryCloseToGoal");
    newGoalDistance = VERY_CLOSE;
  }
  else if (HysterUtils::comparePossibleNegatives(goalDistance == CLOSE, ballToGoalDistance, 1000.f, 200.f))
  {
    logger.addNote("CloseToGoal");
    newGoalDistance = CLOSE;
  }
  else if (HysterUtils::comparePossibleNegatives(goalDistance == NORMAL, ballToGoalDistance, 2300.f, 200.f))
  {
    logger.addNote("NormalToGoal");
    newGoalDistance = NORMAL;
  }
  else if (HysterUtils::comparePossibleNegatives(goalDistance == FAR, ballToGoalDistance, 4000.f, 200.f))
  {
    logger.addNote("FarFromGoal");
    newGoalDistance = FAR;
  }
  else
  {
    logger.addNote("VeryFarFromGoal");
    newGoalDistance = VERY_FAR;
  }

  goalDistance = newGoalDistance;
  /* Might be a good Idea if the different distances are actually used
  if (newGoalDistance != goalDistance)
  {
    goalDistance = newGoalDistance;
    kickManager.stop();
  }
  */
}

bool GoalObjective::isVeryCloseToGoal(const Vector2f& leftGoalPostPosition, const Vector2f& rightGoalPostPosition, const float ballToGoalDistance)
{
  const bool hysteresis = goalDistance == VERY_CLOSE;
  if (!HysterUtils::comparePossibleNegatives(hysteresis, ballToGoalDistance, 200.f, 100.f))
  {
    return false;
  }

  if (!HysterUtils::comparePossibleNegatives(hysteresis, role->ballPosition.y(), leftGoalPostPosition.y(), 0.f, 50.f))
  {
    return false;
  }

  if (!HysterUtils::comparePossibleNegatives(hysteresis, rightGoalPostPosition.y(), role->ballPosition.y(), 0.f, 50.f))
  {
    return false;
  }

  return true;
}

bool GoalObjective::kickToGoalCenter(Ballchaser& ballchaser)
{
  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal - role->theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal + role->theFieldDimensions.goalPostRadius};
  Optimize optimize;
  switch (role->danger)
  {
  case HIGH:
  case MEDIUM:
  case LOW:
    optimize = forTime;
    break;
  case NONE:
  case IMPOSSIBLE:
    optimize = forPrecisionWithoutDanger;
    break;
  default:
    throw std::logic_error("");
  }
  bool suc = kickManager.kickInRangeCenter(ballchaser,
      role->theRobotPoseAfterPreview.translation,
      role->ballPosition,
      leftGoalPostPosition,
      rightGoalPostPosition,
      10,
      DistanceRequirement::shouldFurther,
      optimize,
      100.f,
      0.f,
      KickUtils::unpack(kicks),
      role->theFieldDimensions,
      role->theRobotMap);
  logger.addReason(suc, "KickInRangeCenter");
  return suc;
}

bool GoalObjective::kickBestToGoal(Ballchaser& ballchaser)
{
  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal - role->theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal + role->theFieldDimensions.goalPostRadius};

  Vector2f leftTarget;
  Vector2f rightTarget;
  switch (goalDistance)
  {
  case VERY_CLOSE:
  {
    leftTarget = {leftGoalPostPosition.x() + 300.f, leftGoalPostPosition.y()};
    rightTarget = {rightGoalPostPosition.x() + 300.f, rightGoalPostPosition.y()};
    break;
  }
  case CLOSE:
  case NORMAL:
  case FAR:
  case VERY_FAR:
  {
    leftTarget = leftGoalPostPosition;
    rightTarget = rightGoalPostPosition;
    break;
  }
  default:
    throw std::logic_error("");
  }

  DistanceRequirement distanceRequirement = DistanceRequirement::mustFurther;
  float timeFactor;
  switch (role->danger)
  {
  case HIGH:
    distanceRequirement = DistanceRequirement::mayShorterOrFurther;
    timeFactor = -2.f;
    break;
  case MEDIUM:
    timeFactor = -1.5f;
    break;
  case LOW:
    timeFactor = -1.f;
    break;
  case NONE:
    timeFactor = -0.5f;
    break;
  case IMPOSSIBLE:
    timeFactor = 0.f;
    break;
  default:
    throw std::logic_error("");
  }

  ExecutableKicks executableKicks =
      kickManager
          .getExecutableKicks(
              role->theRobotPoseAfterPreview, {role->ballPosition, leftTarget, rightTarget, distanceRequirement}, 10, KickUtils::unpack(kicks), role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap)
          .filterBlocked(role->theFieldDimensions, role->theRobotMap)
          .reduceToBest(timeFactor, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, role->kickWithLeft, role->theFieldDimensions, role->theHeatMapCollection, role->theRobotMap);

  if (executableKicks.hasBest())
  {
    ExecutableKick executableKick = executableKicks.getBest();
    kickManager.kickTo(ballchaser, role->theRobotPoseAfterPreview, executableKicks.ballPosition, executableKick.target, *executableKick.kick);
    logger.addSuccessReason("BestGoalKick");
    return true;
  }
  else
  {
    logger.addFailedReason("BestGoalKick");
    return false;
  }
}
