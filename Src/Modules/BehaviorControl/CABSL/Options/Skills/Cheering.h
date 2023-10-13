option(Cheering)
{
  initial_state(inital)
  {
    transition
    {
      if (theOwnTeamInfo.score - theOpponentTeamInfo.score > 0 && theFallDownState.state == FallDownState::upright && state_time < 3000 && theRobotInfo.penalty == PENALTY_NONE
          && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && !theGameInfo.firstHalf)
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
      // TODO start fire eyes

      // Choose a random cheering motion
      float r = randomFloat();
      if (r < 0.20f)
        SpecialAction(SpecialActionRequest::cheering4);
      else if (r < 0.50f)
        SpecialAction(SpecialActionRequest::cheering3);
      else
        SpecialAction(SpecialActionRequest::cheering2);
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
