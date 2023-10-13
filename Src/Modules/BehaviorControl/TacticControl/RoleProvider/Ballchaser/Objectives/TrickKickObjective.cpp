#include "TrickKickObjective.h"

#include <Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h>
#include <Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Functions/SelectFunctions.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Factors.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Filterer/Filterer.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Utils/TacticUtils.h>

TickKickObjective::TickKickObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("TrickKickObjective", role, logger), kickManager()
{
  auto kickEngineParameters = KicksProvider::loadKickEngineParameters();
  auto customStepFiles = KicksProvider::loadCustomStepFiles();
  kicks = KicksProvider::createKicks(kickEngineParameters, customStepFiles, role->oneVsOneObjectiveTrickKickNames);
}

bool TickKickObjective::enterCondition()
{
  logger.startInner("enter");

  if (role->theTacticSymbols.closeToBallOpponentRobotNumber == 0)
  {
    logger.addNote("NoOpponentsCloseToBall");
    logger.stopInner();
    return false;
  }
  else
  {
    logger.addNote("OpponentsCloseToBall");
  }

  if (!isBlockingImportantKickForOpponent())
  {
    logger.addNote("NotBlocking");
    logger.stopInner();
    return false;
  }
  else
  {
    logger.addNote("Blocking");
  }

  logger.stopInner();
  return true;
}

bool TickKickObjective::isBlockingImportantKickForOpponent()
{
  const Pose2f& playerPose = role->theRobotPoseAfterPreview;
  const Vector2f& ballPosition = role->theBallSymbols.ballPositionField;
  const Vector2f& opponentPosition = role->theTacticSymbols.closeToBallOpponentRobot.translation;
  const Vector2f& opponentToBall = ballPosition - opponentPosition;
  const Cone defensiveCone = TacticUtils::getDefenseCone(100_deg, role->ballPosition, role->theFieldDimensions);

  // Draw
  const Vector2f leftAnglePoint = ballPosition + MathUtils::angleToVector(defensiveCone.left).normalized(1000.f);
  const Vector2f rightAnglePoint = ballPosition + MathUtils::angleToVector(defensiveCone.right).normalized(1000.f);
  LINE(DRAW_KICK_RANGE_NAME, ballPosition.x(), ballPosition.y(), leftAnglePoint.x(), leftAnglePoint.y(), 20, Drawings::solidPen, ColorRGBA::red);
  LINE(DRAW_KICK_RANGE_NAME, ballPosition.x(), ballPosition.y(), rightAnglePoint.x(), rightAnglePoint.y(), 20, Drawings::solidPen, ColorRGBA::green);

  // Is opponent playing in wrong direction
  const Angle opponentAngle = (ballPosition - opponentPosition).angle();
  const Angle opponentLeftAngle = Angle(180_deg + defensiveCone.right).normalize();
  const Angle opponentRightAngle = Angle(180_deg + defensiveCone.left).normalize();
  if (!MathUtils::isBetweenAngles(opponentAngle, opponentLeftAngle, opponentRightAngle))
  {
    logger.addNote("OpponentWrongSideOfDefenseLine");
    return false;
  }

  // Is player playing in wrong direction
  const Angle playerAngle = (playerPose.translation - ballPosition).angle();
  if (!defensiveCone.istInside(playerAngle))
  {
    logger.addNote("PlayerWrongSideOfDefenseLine");
    return false;
  }

  // Is opponent target outside field
  const float OPPONENT_POSSIBLE_KICK_DISTANCE = 1500.f;
  const Vector2f opponentsPossibleTarget = ballPosition + opponentToBall.normalized(OPPONENT_POSSIBLE_KICK_DISTANCE);
  if (!role->theFieldDimensions.isBallInsideField(opponentsPossibleTarget))
  {
    logger.addNote("OpponentTargetOutsideField");
    return false;
  }

  // Is player standing in opponents kick cone
  const float ROBOT_RADIUS = 150.f;
  const Vector2f opponentLeftSide = opponentPosition + opponentToBall.normalized(ROBOT_RADIUS).rotateLeft();
  const Vector2f opponentRightSide = opponentPosition + opponentToBall.normalized(ROBOT_RADIUS).rotateRight();
  const Angle opponentsConeLeftAngle = (ballPosition - opponentRightSide).angle();
  const Angle opponentsConeRightAngle = (ballPosition - opponentLeftSide).angle();
  const Angle ballToPlayerAngle = (playerPose.translation - ballPosition).angle();
  //
  const Vector2f opponentLeftSideEndpoint = opponentLeftSide + (ballPosition - opponentLeftSide).normalized(3000.f);
  const Vector2f opponentRightSideEndpoint = opponentRightSide + (ballPosition - opponentRightSide).normalized(3000.f);
  LINE(DRAW_KICK_RANGE_NAME, opponentLeftSide.x(), opponentLeftSide.y(), opponentLeftSideEndpoint.x(), opponentLeftSideEndpoint.y(), 20, Drawings::solidPen, ColorRGBA::blue);
  LINE(DRAW_KICK_RANGE_NAME, opponentRightSide.x(), opponentRightSide.y(), opponentRightSideEndpoint.x(), opponentRightSideEndpoint.y(), 20, Drawings::solidPen, ColorRGBA::blue);
  //
  if (!MathUtils::isBetweenAngles(ballToPlayerAngle, opponentsConeLeftAngle, opponentsConeRightAngle))
  {
    logger.addNote("PlayerNotInOpponentsKickCone");
    return false;
  }

  return true;
}

bool TickKickObjective::perform(Ballchaser& ballchaser)
{
  if (performTrickKick(ballchaser))
  {
    logger.addSuccessReason("TrickKick");
    return true;
  }
  else
  {
    logger.addFailedReason("TrickKick");
    return false;
  }
}

/**
 * Stand in front of opponent and try tricks with angled kicks
 */
bool TickKickObjective::performTrickKick(Ballchaser& ballchaser)
{
  const Pose2f playerPose = role->theRobotPoseAfterPreview;
  const Vector2f ballPosition = role->theBallSymbols.ballPositionField;
  const auto [leftAngle, rightAngle] = TacticUtils::getDefenseCone(180_deg, role->ballPosition, role->theFieldDimensions);

  const Filterer filterer =
      Filterer()
          //.filterTooHighRotationToKick(playerPose.rotation, 15_deg)
          .filterOutside()
          //.filterBlocked(role->theKickWheel)
          .filterBetweenAngles(role->ballPosition, leftAngle, rightAngle);
  const Factors factors = {0.f, 0.f, 1.f, 1.f, 1.f, -1.f, -1.f, 1.f, 0.f, 1.f, -10.f, 0.f};
  auto selectableKickOptional = SelectFunctions::createAndFilterAndSelect(role->theRobotPoseAfterPreview,
      role->ballPosition,
      KickUtils::unpack(kicks),
      kickManager.getCurrentKick(role->ballPosition),
      filterer,
      factors,
      role->theFieldDimensions,
      role->theHeatMapCollection,
      role->theKickWheel,
      role->theRobotMap,
      role->theTacticSymbols);
  if (selectableKickOptional.has_value())
  {
    kickManager.kickTo(ballchaser, selectableKickOptional.value(), role->theFrameInfo);
    return true;
  }
  else
  {
    kickManager.stop();
    return false;
  }
}

bool TickKickObjective::leaveCondition() const
{
  const float distanceToBall = Geometry::distance(role->ballPosition, role->theRobotPoseAfterPreview.translation);
  if (distanceToBall > 500.f)
  {
    return true;
  }
  if (KickUtils::isBallKicked(role->theMotionInfo))
  {
    return true;
  }
  return false;
}

void TickKickObjective::postprocess()
{
  Objective::postprocess();
  kickManager.stop();
}
