option(Cheering)
{
  initial_state(inital)
  {
    transition
    {
      if (theOwnTeamInfo.score - theOpponentTeamInfo.score > 0 && theFallDownState.state == FallDownState::upright && state_time < 3000 && theRobotInfo.penalty == PENALTY_NONE
          && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
        goto cheering;
      else
        goto sit;
    }
    action {}
  }

  state(cheering)
  {
    transition
    {
      if (action_done)
        goto sit;
    }
    action
    {
      // Choose a random cheering motion
      float r = randomFloat();
      if (r < 0.33f)
        SpecialAction(SpecialActionRequest::cheering3);
      else if (r < 0.66f)
        SpecialAction(SpecialActionRequest::cheering2);
      else
        SpecialAction(SpecialActionRequest::cheering1);
    }
  }

  state(sit)
  {
    transition {}
    action
    {
      SpecialAction(SpecialActionRequest::sitDown);
    }
  }
}
