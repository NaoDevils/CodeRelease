#include "ExecuteRecommendationObjective.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/KickTypes/Dribble.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/KicksProvider.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserUtils.h"

ExecuteRecommendationObjective::ExecuteRecommendationObjective(BallchaserProvider* role, BehaviorLogger& logger) : Objective("ExecuteRecommendationObjective", role, logger) {}

bool ExecuteRecommendationObjective::perform(Ballchaser& ballchaser)
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

bool ExecuteRecommendationObjective::leaveCondition() const
{
  return true;
}

void ExecuteRecommendationObjective::postprocess()
{
  Objective::postprocess();
}
