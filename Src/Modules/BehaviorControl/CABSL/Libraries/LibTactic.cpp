#include "../LibraryBase.h"
#include <algorithm>
#include <float.h>


namespace NDBehavior
{
#include "LibTactic.h"

  LibTactic::LibTactic() {}

  void LibTactic::preProcess()
  {
    /* Option Playing */
    /* Check if interception would be possible
    

    Vector2f endPosition = BallPhysics::getEndPosition(Vector2f::Zero(), theBallSymbols.ballVelocityRelative, theBallModel.friction);

    interceptBallPossible = theBehaviorConfiguration.behaviorParameters.useBallInterception
    && theBallSymbols.ballWasSeen
    && std::abs(theBallSymbols.yPosWhenBallReachesOwnYAxis) < 500.f
    && (theBallSymbols.ballPositionRelativePredicted.x() < theBehaviorConfiguration.behaviorParameters.minXPositionForIntercept || endPosition.x() < -1500.f)
    && theBallSymbols.ballVelocityRelative.x() < -200.f;*/


    /*Calculate kickWithOuterFoot*/
    Angle kickDirection = theRobotPoseAfterPreview.rotation + theBallSymbols.ballPositionRelative.angle();
    kickDirection = kickDirection.normalize();

    Pose2f ballPositionAfterKick((theBallSymbols.ballPositionField - theRobotPoseAfterPreview.translation).angle(), theBallSymbols.ballPositionField);
    ballPositionAfterKick.translate(1500.f, 0.f);
    bool ballIsOut = !theFieldDimensions.isInsideField(ballPositionAfterKick.translation);
    bool ownKickoffSetPlay = theGameSymbols.ownKickOff && theGameSymbols.currentSetPlay != SET_PLAY_NONE;

    kickWithOuterFoot = (theBehaviorConfiguration.behaviorParameters.useBlindSideKick //Aktuelle Position
        && !ownKickoffSetPlay && theBallSymbols.timeSinceLastSeen < 2000 && std::abs(theBallSymbols.ballPositionRelative.x()) < theBehaviorConfiguration.behaviorParameters.relativeXBallPosition
        && std::abs(theBallSymbols.ballPositionRelative.y()) > theBehaviorConfiguration.behaviorParameters.relativeYminBallPosition
        && std::abs(theBallSymbols.ballPositionRelative.y()) < theBehaviorConfiguration.behaviorParameters.relativeYmaxBallPosition && !ballIsOut
        && std::abs(kickDirection) < 100_deg && theFrameInfo.getTimeSince(timeStampLastWalkKickExecution) > 2000 && theFrameInfo.getTimeSince(timeStampLastDribbleExecution) > 2000
        && theSpeedInfo.stepsSinceLastCustomStep > theBehaviorConfiguration.behaviorParameters.minStepsBetweenWalkKicks);
  }

  void LibTactic::postProcess() {}
} // namespace NDBehavior
