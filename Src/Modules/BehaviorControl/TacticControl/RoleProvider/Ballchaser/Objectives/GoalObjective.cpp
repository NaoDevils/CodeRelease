#include "GoalObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/TargetFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/KickLine.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/HysteresisUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/KickCone.h>

GoalObjective::GoalObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("GoalObjective", role, logger), kickManager()
{
  auto kickEngineParameters = KicksProvider::loadKickEngineParameters();
  auto customStepFiles = KicksProvider::loadCustomStepFiles();
  noDangerKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, role->goalObjectiveNoDangerKickNames);
  dangerKicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, role->goalObjectiveDangerKickNames);
}

bool GoalObjective::enterCondition()
{
  const Angle MAX_ANGLE = 74_deg; // TODO Constant also used in ExecutableKicks to filter
  const bool leftAngleTooSharp = HysteresisUtils::compareAbsolutes(openingAngleTooSharp, MAX_ANGLE, -role->theGoalSymbols.leftAngleBallToOppGoalWC, 2_deg);
  const bool rightAngleTooSharp = HysteresisUtils::compareAbsolutes(openingAngleTooSharp, MAX_ANGLE, role->theGoalSymbols.rightAngleBallToOppGoalWC, 2_deg);
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
  updateGoalDistance();

  switch (goalDistance)
  {
  case VERY_FAR:
  {
    logger.addFailedReason("TooFar");
    return false;
  }
  case FAR:
  {
    return kickToGoalCenter(ballchaser);
  }
  case NORMAL:
  case CLOSE:
  case VERY_CLOSE:
    return kickBestToGoal(ballchaser);
  default:
    throw std::runtime_error("Unknown Enum Value!");
  }
}

void GoalObjective::updateGoalDistance()
{
  const float ballToGoalLineDistance = FieldUtils::getDistanceToGoalLine(role->ballPosition, role->theFieldDimensions);
  GoalDistance newGoalDistance;
  if (isVeryCloseToGoal())
  {
    logger.addNote("VeryCloseToGoal");
    newGoalDistance = VERY_CLOSE;
  }
  else if (HysteresisUtils::comparePossibleNegatives(goalDistance == CLOSE, ballToGoalLineDistance, 1000.f, 200.f))
  {
    logger.addNote("CloseToGoal");
    newGoalDistance = CLOSE;
  }
  else if (HysteresisUtils::comparePossibleNegatives(goalDistance == NORMAL, ballToGoalLineDistance, 2300.f, 200.f))
  {
    logger.addNote("NormalToGoal");
    newGoalDistance = NORMAL;
  }
  else if (HysteresisUtils::comparePossibleNegatives(goalDistance == FAR, ballToGoalLineDistance, 5000.f, 200.f))
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
}

bool GoalObjective::isVeryCloseToGoal() const
{
  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal};

  const bool hysteresis = goalDistance == VERY_CLOSE;

  if (!HysteresisUtils::comparePossibleNegatives(hysteresis, role->ballPosition.y(), leftGoalPostPosition.y(), 0.f, 50.f))
  {
    return false;
  }

  if (!HysteresisUtils::comparePossibleNegatives(hysteresis, rightGoalPostPosition.y(), role->ballPosition.y(), 0.f, 50.f))
  {
    return false;
  }

  const Geometry::Line goalLine = {leftGoalPostPosition, (rightGoalPostPosition - leftGoalPostPosition).normalized()};
  const float ballToGoalDistance = Geometry::getDistanceToLine(goalLine, role->ballPosition);
  return HysteresisUtils::comparePossibleNegatives(hysteresis, ballToGoalDistance, 200.f, 100.f);
}

bool GoalObjective::kickToGoalCenter(Ballchaser& ballchaser)
{
  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal - role->theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal + role->theFieldDimensions.goalPostRadius};

  bool optimizeForTime;
  std::vector<Kick*> kicks;
  switch (role->danger)
  {
  case Danger::HIGH:
    optimizeForTime = true;
    kicks = KickUtils::unpack(dangerKicks);
    logger.addNote("UseDangerKicks");
    break;
  case Danger::MEDIUM:
  case Danger::LOW:
  case Danger::NONE:
  case Danger::IMPOSSIBLE:
    optimizeForTime = false;
    kicks = KickUtils::unpack(noDangerKicks);
    logger.addNote("UseNoDangerKicks");
    break;
  default:
    throw std::logic_error("");
  }

  const Vector2f goalCenter = (leftGoalPostPosition + rightGoalPostPosition) / 2;
  const float ballToGoalCenterDistance = Geometry::distance(goalCenter, role->ballPosition);
  std::vector<Kick*> enoughRangeKicks;
  for (auto kick : kicks)
  {
    if (kick->getRealisticDistance() > ballToGoalCenterDistance)
    {
      enoughRangeKicks.push_back(kick);
    }
  }

  const KickCone kickCone = {role->ballPosition, (leftGoalPostPosition - role->ballPosition).angle(), (rightGoalPostPosition - role->ballPosition).angle()};
  const Filterer filterer = Filterer().filterBlocked().filterOutsideKickRange(kickCone);
  std::function<float(KickPlan&)> customScoreFunction = [&leftGoalPostPosition, &goalCenter](const KickPlan& kickPlan)
  {
    const Geometry::Line goalLine = {leftGoalPostPosition, goalCenter - leftGoalPostPosition};
    const Geometry::Line kickLine = {
        kickPlan.selectablePose.selectableTarget.ballPosition, kickPlan.selectablePose.selectableTarget.target - kickPlan.selectablePose.selectableTarget.ballPosition};
    Vector2f intersection;
    VERIFY(Geometry::getIntersectionOfLines(goalLine, kickLine, intersection));
    const float distanceToCenter = std::abs(intersection.y()); // center is at y = 0
    const float maxDistanceToCenter = leftGoalPostPosition.y(); // center is at y = 0
    return 2 * (1 - distanceToCenter / maxDistanceToCenter);
  };
  const Factors factors = Factors(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, optimizeForTime ? -1.f : 0.f, optimizeForTime ? 0.f : -1.f).addCustomScoreFunction(customScoreFunction);
  const auto selectableKickOptional = SelectFunctions::createAndFilterAndSelect(role->theRobotPoseAfterPreview,
      role->ballPosition,
      enoughRangeKicks,
      kickManager.getCurrentKick(role->ballPosition),
      filterer,
      factors,
      role->theFieldDimensions,
      role->theHeatMapCollection,
      role->theKickWheel,
      role->theRobotMap,
      role->theTacticSymbols);

  const bool suc = selectableKickOptional.has_value();
  if (suc)
  {
    kickManager.kickTo(ballchaser, selectableKickOptional.value(), role->theFrameInfo);
  }
  logger.addReason(suc, "KickInRangeCenter");
  return suc;
}

bool GoalObjective::kickBestToGoal(Ballchaser& ballchaser)
{
  const Vector2f leftGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal - role->theFieldDimensions.goalPostRadius};
  const Vector2f rightGoalPostPosition = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal + role->theFieldDimensions.goalPostRadius};

  Vector2f leftTarget = leftGoalPostPosition;
  Vector2f rightTarget = rightGoalPostPosition;

  float widthFactor = -1000.f; // These values will lead to an assertion error if not overwritten
  float timeFactor = 1000.f;
  std::vector<Kick*> kicks;
  switch (goalDistance)
  {
  case VERY_CLOSE:
  {
    leftTarget = {leftGoalPostPosition.x() + 150.f, leftGoalPostPosition.y()};
    rightTarget = {rightGoalPostPosition.x() + 150.f, rightGoalPostPosition.y()};
    widthFactor = 0.f; // Important, so the target doesn't change if the goalKeeper comes close
    logger.addNote("VeryClose->IgnoreKickWidth");
    break;
  }
  case CLOSE:
  {
    switch (role->danger)
    {
    case Danger::HIGH:
    case Danger::MEDIUM:
      widthFactor = 0.f; // Important, so the target doesn't change if the goalKeeper comes close
      logger.addNote("CloseAndHighDanger->IgnoreKickWidth");
      break;
    case Danger::LOW:
    case Danger::NONE:
    case Danger::IMPOSSIBLE:
      widthFactor = 0.5f;
      break;
    default:
      throw std::logic_error("");
    }
    break;
  }
  case NORMAL:
    widthFactor = 1.f;
    break;
  case FAR:
    widthFactor = 1.f;
    break;
  case VERY_FAR:
    widthFactor = 1.f;
    break;
  default:
    throw std::logic_error("");
  }
  switch (role->danger)
  {
  case Danger::HIGH:
    timeFactor = -2.f;
    kicks = KickUtils::unpack(dangerKicks);
    logger.addNote("OnlyDangerKicks");
    break;
  case Danger::MEDIUM:
    timeFactor = -1.f;
    kicks = KickUtils::unpack(noDangerKicks);
    logger.addNote("OnlyDangerKicks");
    break;
  case Danger::LOW:
    timeFactor = -0.2f;
    kicks = KickUtils::unpack(noDangerKicks);
    logger.addNote("OnlyNoDangerKicks");
    break;
  case Danger::NONE:
    timeFactor = -0.1f;
    kicks = KickUtils::unpack(noDangerKicks);
    logger.addNote("OnlyNoDangerKicks");
    break;
  case Danger::IMPOSSIBLE:
    timeFactor = 0.f;
    kicks = KickUtils::unpack(noDangerKicks);
    logger.addNote("OnlyNoDangerKicks");
    break;
  default:
    throw std::logic_error("");
  }

  const KickLine kickLine = {role->ballPosition, leftTarget, rightTarget, mustFurther, 300.f};
  const Filterer filterer = Filterer().filterOutsideKickRange(kickLine).filterBlocked();
  const Factors factors = {widthFactor, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.1f, 0.f, 0.f, timeFactor, 0.f};
  const auto selectableKickOptional = SelectFunctions::createAndFilterAndSelect(
      role->theRobotPoseAfterPreview, role->ballPosition, kicks, kickManager.getCurrentKick(role->ballPosition), filterer, factors, role->theFieldDimensions, role->theHeatMapCollection, role->theKickWheel, role->theRobotMap, role->theTacticSymbols);

  if (selectableKickOptional.has_value())
  {
    kickManager.kickTo(ballchaser, selectableKickOptional.value(), role->theFrameInfo);
    logger.addSuccessReason("BestGoalKick");
    return true;
  }
  else
  {
    logger.addFailedReason("BestGoalKick");
    return false;
  }
}

void GoalObjective::postprocess()
{
  kickManager.stop();
  Objective::postprocess();
}
