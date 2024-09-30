#include "RecommendedKickObjective.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h"

RecommendedKickObjective::RecommendedKickObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("RecommendedKickObjective", role, logger) {}

bool RecommendedKickObjective::perform(Ballchaser& ballchaser)
{
  if (role->theRecommendedKick.hasRecommendation)
  {
    // positioning symbols
    ballchaser.optPosition = role->theRecommendedKick.optPosition;
    ballchaser.thresholdXFront = role->theRecommendedKick.thresholdXFront;
    ballchaser.thresholdXBack = role->theRecommendedKick.thresholdXBack;
    ballchaser.thresholdY = role->theRecommendedKick.thresholdY;
    ballchaser.thresholdRotation = role->theRecommendedKick.thresholdRotation;
    // positioning and kick symbols
    ballchaser.kickType = role->theRecommendedKick.kickType;
    ballchaser.walkKickType = role->theRecommendedKick.walkKickType;
    ballchaser.longKickType = role->theRecommendedKick.longKickType;
    ballchaser.mirrorKick = role->theRecommendedKick.mirrorKick;
    ballchaser.useFastKick = role->theRecommendedKick.useFastKick;
    ballchaser.kickBlind = role->theRecommendedKick.kickBlind;
    ballchaser.kickTarget = role->theRecommendedKick.kickTarget;

    logger.addSuccessReason("KickBestWithFactors");
    return true;
  }
  else
  {
    logger.addFailedReason("KickBestWithFactors");
    logger.addSuccessReason("Defend");
    const Vector2f defensivePosition = BallchaserUtils::getWaitPosition(role->theBallSymbols.ballPositionFieldPredicted, role->theFieldDimensions);
    PositionUtils::setPosition(ballchaser, defensivePosition);
    return true;
  }
}

bool RecommendedKickObjective::leaveCondition() const
{
  return true;
}

void RecommendedKickObjective::postprocess()
{
  Objective::postprocess();
}
