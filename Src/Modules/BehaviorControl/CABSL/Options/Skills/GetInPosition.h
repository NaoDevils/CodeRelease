option(GetInPosition)
{
  initial_state(positioning)
  {
    transition
    {
      // keeper should not be off by 40cm (see below) during penalty kicks/shootout
      const bool isPenaltyKeeper = (theRoleSymbols.role == BehaviorData::RoleAssignment::keeper || theRoleSymbols.role == BehaviorData::RoleAssignment::replacementKeeper)
          && (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT);
      if (action_done && !isPenaltyKeeper)
        goto waitForSet;
    }
    action
    {
      GoToFieldCoordinates(
          thePositioningSymbols.optPosition, thePositioningSymbols.thresholdXFront, thePositioningSymbols.thresholdXBack, thePositioningSymbols.thresholdY, thePositioningSymbols.thresholdRotation, true, true);
    }
  }

  state(waitForSet)
  {
    transition
    {
      if (thePositioningSymbols.distanceToOptPosition >= 400.f || thePositioningSymbols.inIllegalPosition)
        goto positioning;
    }
    action
    {
      Stand();
    }
  }
}
