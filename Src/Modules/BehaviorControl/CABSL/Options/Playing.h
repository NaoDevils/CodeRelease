/**
* An option handling the playing state.
*
* Checks if a special behavior for the current situation is needed or simple positioning is enough.
* Actual execution of behavior is handled in seperate options.
*/
option(Playing)
{
  common_transition
  {
    if (libTactic.interceptBallPossible && theRoleSymbols.role != BehaviorData::RoleAssignment::keeper && theRoleSymbols.role == BehaviorData::RoleAssignment::replacementKeeper)
      goto interceptBall;
    // some roles have special behavior handled in their own options
    switch (theRoleSymbols.role)
    {
    case BehaviorData::ballchaser:
      // chase the ball and try to score a goal
      goto ballchaser;
      break;
    case BehaviorData::keeper:
      // defend the own goal
      goto keeper;
      break;
    case BehaviorData::replacementKeeper:
      // defend own goal as field player
      goto replacementKeeper;
      break;
    default:
      // no special role behavior needed => walk to position given in PositioningSymbols
      goto positioning;
    }
  }

  /**
  * Initial state immediately left through common_transition.
  */
  initial_state(checkForRoleBehavior)
  {
    transition
    {
      // handled by common transition
    }
  }

  state(interceptBall)
  {
    transition {}
    action
    {
      InterceptBall(false);
      if (state_time == 0)
        SystemCall::playSound("doh.wav");
    }
  }

  /**
  * Walk to the position provided by the PositioningSymbols.
  *
  * The PositioningSymbols will automatically be set to the optimal position for the current role
  * so this state actually handled role specific behavior.
  */
  state(positioning)
  {
    action
    {
      theBehaviorData.soccerState = BehaviorData::positioning;
      Positioning();
    }
  }

  /**
  * Handles special ballchaser behavior.
  */
  state(ballchaser)
  {
    action
    {
      theBehaviorData.soccerState = BehaviorData::controlBall;
      Ballchaser();
    }
  }

  /**
  * Handles special keeper behavior.
  */
  state(keeper)
  {
    action
    {
      theBehaviorData.soccerState = BehaviorData::positioning;
      Keeper();
    }
  }

  /**
  * Handles special replacementKeeper behavior.
  */
  state(replacementKeeper)
  {
    action
    {
      theBehaviorData.soccerState = BehaviorData::positioning;
      ReplacementKeeper();
    }
  }
}
