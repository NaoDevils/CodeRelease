#include "MoveObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>

MoveObjective::MoveObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("MoveObjective", role, logger)
{
  const auto kickEngineParameter = KicksProvider::loadKickEngineParameters();
  const auto customStepFiles = KicksProvider::loadCustomStepFiles();
  noDangerKicks = KicksProvider::createKicks(kickEngineParameter, customStepFiles, role->moveObjectiveNoDangerKickNames);
  dangerKicks = KicksProvider::createKicks(KicksProvider::loadKickEngineParameters(), KicksProvider::loadCustomStepFiles(), role->moveObjectiveDangerKickNames);
}

bool MoveObjective::perform(Ballchaser& ballchaser)
{
  auto kicks = getKicks(role->danger);

  const Vector2f opponentGoalCenter = FieldUtils::getOpponentGoalCenter(role->theFieldDimensions);
  const float distanceToOpponentGoalCenter = FieldUtils::getDistanceToGoalLine(role->ballPosition, role->theFieldDimensions);
  const Vector2f ownLeftGoalPost = {role->theFieldDimensions.xPosOwnGroundline, role->theFieldDimensions.yPosLeftGoal};
  const Vector2f ownRightGoalPost = {role->theFieldDimensions.xPosOwnGroundline, role->theFieldDimensions.yPosRightGoal};
  const Vector2f opponentLeftGoalPost = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosLeftGoal};
  const Vector2f opponentRightGoalPost = {role->theFieldDimensions.xPosOpponentGroundline, role->theFieldDimensions.yPosRightGoal};

  Filterer filterer = {};

  if (distanceToOpponentGoalCenter < 50.f) // TODO Hysteresis
  {
    filterer.filterLeft(opponentLeftGoalPost.y()).filterRight(opponentRightGoalPost.y()).filterTooFarBack(role->ballPosition.x());
  }
  else
  {
    const float OWN_SIDE_X = role->theFieldDimensions.xPosOwnPenaltyArea;
    const float OPPONENT_SIDE_X = (0.f + role->theFieldDimensions.xPosOpponentPenaltyArea) / 2.f;

    Vector2f linePoint1 = role->ballPosition;
    Vector2f linePoint2;

    Vector2f ownSideMoveInDirection = {1.f, 0};
    Vector2f opponentSideMoveInDirection = opponentGoalCenter - role->ballPosition;

    const bool ballOnOpponentSide = role->ballPosition.x() > OPPONENT_SIDE_X;
    const bool ballBetweenSides = !ballOnOpponentSide && role->ballPosition.x() > OWN_SIDE_X;

    if (ballOnOpponentSide)
    {
      linePoint2 = role->ballPosition + Vector2f(opponentSideMoveInDirection).rotateLeft();
      if (Geometry::isPointLeftOfLine(opponentLeftGoalPost, linePoint1, linePoint2))
      {
        linePoint2 = opponentLeftGoalPost;
      }
      else if (Geometry::isPointLeftOfLine(opponentRightGoalPost, linePoint1, linePoint2))
      {
        linePoint2 = linePoint1;
        linePoint1 = opponentRightGoalPost;
      }
    }
    else if (ballBetweenSides)
    {
      const float opponentSideFactor = (role->ballPosition.x() - OWN_SIDE_X) / (OPPONENT_SIDE_X - OWN_SIDE_X);
      const Vector2f moveInDirection = (1 - opponentSideFactor) * ownSideMoveInDirection + opponentSideFactor * opponentSideMoveInDirection;
      linePoint2 = role->ballPosition + Vector2f(moveInDirection).rotateLeft();
    }

    if (ballOnOpponentSide || ballBetweenSides) // TODO Hysteresis!?
    {
      filterer.filterTooFarBack(std::min(role->ballPosition.x() - 500.f, role->theFieldDimensions.xPosOpponentPenaltyArea)).filterLeftOfLine(linePoint1, linePoint2);
    }
    else
    {
      filterer.filterToOwnGoal(role->ballPosition, role->theFieldDimensions);
    }
  }

  filterer.filterOutside()
      .filterKickPoseBlockedByGoalPost(role->theFieldDimensions)
      .filterKickPoseBlockedByRobots(role->theRobotMap)
      .filterInFrontOfOwnGoal(role->theFieldDimensions)
      .filterCloseToFieldLine(role->theFieldDimensions);

  const Factors factors = {getWidthFactor(role->danger),
      0.f,
      1.f, // Field
      0.01f,
      0.2f, // Team
      -0.01f,
      -0.2f, // Opponent
      0.1f,
      0.f,
      0.f, // Pose
      getTimeFactor(role->danger),
      0.f}; // Kick

  const std::optional<KickPlan> kickPlanOptional = SelectFunctions::createAndFilterAndSelect(
      role->theRobotPoseAfterPreview, role->ballPosition, kicks, kickManager.getCurrentKick(role->ballPosition), filterer, factors, role->theFieldDimensions, role->theHeatMapCollection, role->theKickWheel, role->theRobotMap, role->theTacticSymbols);

  if (kickPlanOptional.has_value())
  {
    kickManager.kickTo(ballchaser, kickPlanOptional.value(), role->theFrameInfo);
    logger.addSuccessReason("KickBestWithFactors");
    return true;
  }
  else
  {
    logger.addFailedReason("KickBestWithFactors");
    logger.addSuccessReason("Defend");
    const Vector2f defensivePosition = BallchaserUtils::getWaitPosition(role->ballPosition, role->theFieldDimensions);
    PositionUtils::setPosition(ballchaser, defensivePosition);
    return true;
  }
}

std::vector<Kick*> MoveObjective::getKicks(const Danger danger)
{
  switch (danger)
  {
  case Danger::HIGH:
    return KickUtils::unpack(dangerKicks);
  case Danger::MEDIUM:
  case Danger::LOW:
  case Danger::NONE:
  case Danger::IMPOSSIBLE:
    return KickUtils::unpack(noDangerKicks);
  default:
    throw std::logic_error("");
  }
}

float MoveObjective::getWidthFactor(const Danger danger)
{
  switch (danger) // TODO Use enemy goal shoot probability to calculate danger
  {
  case Danger::HIGH:
    return 0.f;
  case Danger::MEDIUM:
    return 0.f;
  case Danger::LOW:
    return 0.1f;
  case Danger::NONE:
    return 0.15f;
  case Danger::IMPOSSIBLE:
    return 0.15f;
  default:
    throw std::logic_error("");
  }
}

float MoveObjective::getTimeFactor(const Danger danger)
{
  switch (danger) // TODO Use enemy goal shoot probability to calculate danger
  {
  case Danger::HIGH:
    return -1.f;
  case Danger::MEDIUM:
    return -0.5f;
  case Danger::LOW:
    return -0.1f;
  case Danger::NONE:
    return 0.f;
  case Danger::IMPOSSIBLE:
    return 0.f;
  default:
    throw std::logic_error("");
  }
}

bool MoveObjective::leaveCondition() const
{
  return true;
}

void MoveObjective::postprocess()
{
  kickManager.stop();
  Objective::postprocess();
}
