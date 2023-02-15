// Penalty GOALIE

#include "Tools/Math/Pose2f.h"

option(PenaltyKeeper)
{

  // simple decision to dive
  const bool reactionNeeded = thePSGoalieTrigger.hasKicked;
  const bool blockIsEnough = thePSGoalieTrigger.kickDirection == PSGoalieTrigger::KickDirection::CENTER;
  //const bool mirrorDive = thePSGoalieTrigger.kickDirection == PSGoalieTrigger::KickDirection::RIGHT;

  /** STATE MACHINE START **/
  initial_state(esablishStand)
  {
    transition
    {
      if (state_time > 3000 && action_done)
        goto awaitShot;
    }
    action
    {
      SpecialAction(SpecialActionRequest::stand);
    }
  }

  state(awaitShot)
  {
    transition
    {
      if (reactionNeeded)
      {
        if (blockIsEnough)
        {
          goto block;
        }
        else
        {
          goto dive;
        }
      }
    }
    action
    {
      // wait
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
      //SpecialAction(SpecialActionRequest::wideStance);
      ASSERT(false);
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
      //SpecialAction(SpecialActionRequest::goalkeeperDefendPenalty, mirrorDive);
      ASSERT(false);
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
