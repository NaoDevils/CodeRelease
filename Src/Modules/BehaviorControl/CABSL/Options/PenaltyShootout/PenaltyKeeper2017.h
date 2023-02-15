// GOALIE

#include "Tools/Math/Pose2f.h"

option(PenaltyKeeper2017)
{
  const PenaltyShootout2017Parameters::Goalie& parameters = theBehaviorConfiguration.penaltyShootout2017Parameters.goalie;
  const bool useDiveBehavior = parameters.useGoalieDiveBehavior;
  const bool useSpeculative = parameters.useSpeculativeMode;
  const int shooterPositionTime = parameters.checkRobotAtTimeSpeculative;
  const int blockArea = parameters.blockArea;

  // TODO: with button interface no second attemp can be made, only gc then
  int remainingTime = theBehaviorConfiguration.penaltyShootout2017Parameters.striker.durationInSecs - theGameSymbols.timeSincePlayingState / 1000;
  if (static_cast<int>(theGameInfo.secsRemaining) > 0 && static_cast<int>(theGameInfo.secsRemaining) < remainingTime)
    remainingTime = static_cast<int>(theGameInfo.secsRemaining);
  const bool timesUp = remainingTime > 0 && remainingTime < parameters.timeLeftPSForTrigger;

  /*if (timesUp)
    OUTPUT_TEXT("" << remainingTime << "s timesUp!");
  else
    OUTPUT_TEXT("" << remainingTime << "s");*/

  // velocityTrigger deprecated
  //const bool triggerSafeMotionBecauseOfVelocity = (theBallModel.estimate.velocity[0] < parameters.velOfBallForTrigger);
  // TODO: distance trigger should happen if distance is getting smaller
  // or distance is reasonably small, otherwise (like now) its a gamble
  // due to unsafe ball distance after walk (different robots/parameters..)
  const bool triggerSafeMotionBecauseOfDistance =
      parameters.useGoalieDiveBehavior ? (theBallModel.lastPerception.norm() < parameters.distToBallForTriggerDive) : (theBallModel.lastPerception.norm() < parameters.distToBallForTrigger);

  // mirroring of side now done in selfLocator
  /*Pose2f goaliePosition;
  if (theRobotPoseAfterPreview.translation.x() < 0)
    goaliePosition = Pose2f(0_deg, (theFieldDimensions.xPosOwnPenaltyArea - parameters.xOffsetFromPenaltyArea), 0);
  else
    goaliePosition = Pose2f(180_deg, (theFieldDimensions.xPosOpponentPenaltyArea + parameters.xOffsetFromPenaltyArea), 0);*/

  common_transition
  {
    if (theRobotMap.robots.size() > 0)
    {
      Pose2f lastRobot = theRobotMap.robots[0].pose;
      Pose2f nearest = theRobotMap.robots[0].pose;

      for (auto& robot : theRobotMap.robots)
      {
        if ((lastRobot.translation - theBallSymbols.ballPositionField).norm() > (robot.pose.translation - theBallSymbols.ballPositionField).norm())
          nearest = robot.pose;
      }

      Vector2f robotRelative = Transformation::fieldToRobot(theRobotPose, nearest.translation);
      if (robotRelative.y() < theBallSymbols.ballPositionRelative.y())
        mirrorInPenaltyShootout = false;
      else
        mirrorInPenaltyShootout = true;
    }
  }

  /** STATE MACHINE START **/
  initial_state(settle)
  {
    transition
    {
      if (useDiveBehavior || useSpeculative)
        goto wait;
      else if (state_time > 2000)
        goto position;
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(position)
  {
    transition
    {
      if (state_time > parameters.stepForwardDuration)
        goto wait;
    }
    action
    {
      if (theBallSymbols.ballWasSeen && !useSpeculative)
        Walk(WalkRequest::speed, parameters.stepForwardSpeed, 0, theBallSymbols.ballPositionRelative.angle());
      else
        Walk(WalkRequest::speed, parameters.stepForwardSpeed, 0, 0);
    }
  }

  state(wait)
  {
    transition
    {
      if (state_time > 2000)
      {
        if (useDiveBehavior)
        {
          if (triggerSafeMotionBecauseOfDistance /*|| triggerSafeMotionBecauseOfVelocity*/)
          {
            goto blockDecision;
          }
        }
        else if (useSpeculative)
        {
          goto doSpeculative;
        }
        else
        {

          // For debugging which condition triggered the safe motion
          if (timesUp)
          {
            goto timeTriggered;
          }
          // velocityTrigger deprecated
          /*else if (triggerSafeMotionBecauseOfVelocity)
            {
              goto velocityTriggered;
            }*/
          //else
          else if (triggerSafeMotionBecauseOfDistance)
          {
            goto distanceTriggered;
          }
        }
      }
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(doSpeculative)
  {
    transition
    {
      if (state_time > shooterPositionTime)
        goto diveSpeculative;
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  // just for Debugging
  state(timeTriggered)
  {
    transition
    {
      goto block; // wide stance?
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  // velocityTrigger deprecated
  /*state(velocityTriggered)
  {
    transition
    {
        goto block; // wide stance?
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }*/

  state(distanceTriggered)
  {
    transition
    {
      goto block; // wide stance?
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }


  state(blockDecision)
  {
    transition
    {
      if (std::abs(theBallSymbols.yPosWhenBallReachesOwnYAxis) < blockArea)
        goto block;
      else
        goto dive;
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(dive)
  {
    transition
    {
      if ((state_time > 2000) || action_done)
        goto waitForPickup;
    }
    action
    {
      bool mirror = theBallModel.lastPerception.y() < 0;
      if (theBallSymbols.timeUntilBallReachesOwnYAxis < 5000)
        mirror = theBallSymbols.yPosWhenBallReachesOwnYAxis < 0;
      SpecialAction(SpecialActionRequest::goalkeeperDefendPenalty, mirror);
      //SpecialAction(SpecialActionRequest::cheering2, mirror);
    }
  }

  state(diveSpeculative)
  {
    transition
    {
      if (action_done)
        goto waitForPickup;
    }
    action
    {
      SpecialAction(SpecialActionRequest::goalkeeperDefendPenalty, mirrorInPenaltyShootout);
      //SpecialAction(SpecialActionRequest::cheering1, mirror);
    }
  }

  state(block)
  {
    transition
    {
      if ((state_time > 2000) || action_done)
        goto waitForPickup;
    }
    action
    {
      SpecialAction(SpecialActionRequest::wideStance);
      //SpecialAction(SpecialActionRequest::cheering1);
    }
  }

  target_state(waitForPickup)
  {
    transition
    {
      // do nothing
    }
    action
    {
      SpecialAction(SpecialActionRequest::playDead, false);
    }
  }
}
