/** This option handles the ball search of the goalie. */
option(KeeperBallSearch)
{
  initial_state(search_for_ball)
  {
    transition
    {
      if (action_done && state_time > 2000)
      {
        if (theKeeper.ballSearchState == Keeper::KeeperBallSearchState::supporter)
        {
          goto search_for_ball_supporter;
        }
        if (theKeeper.ballSearchState == Keeper::KeeperBallSearchState::movingBall)
        {
          goto search_for_ball_moving;
        }
        if (theKeeper.ballSearchState == Keeper::KeeperBallSearchState::locate)
        {
          goto search_for_ball_symmetry;
        }
      }
      if (theKeeper.ballSearchState == Keeper::KeeperBallSearchState::penalty)
      {
        goto search_for_ball_penalty;
      }
    }
    action
    {
      GoToFieldCoordinates(theKeeper.optPosition, 100, 100, 100, 10_deg, false, true);
      theHeadControlRequest.controlType = HeadControlRequest::soccer;
    }
  }

  state(search_for_ball_moving)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::movingBall)
      {
        goto search_for_ball;
      }
      if (action_done)
      {
        goto search_for_ball_moving_right;
      }
    }
    action
    {
      GoToFieldCoordinates(Pose2f(theKeeper.optPosition.rotation - 45, theKeeper.optPosition.translation), 100, 100, 100, 10_deg, false, true);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = 0_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }

  state(search_for_ball_moving_left)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::movingBall)
      {
        goto search_for_ball;
      }
      if (action_done)
      {
        goto search_for_ball_moving_right;
      }
    }
    action
    {
      GoToFieldCoordinates(Pose2f(theKeeper.optPosition.rotation - 45, theKeeper.optPosition.translation), 100, 100, 100, 10_deg, false, true);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = 0_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }

  state(search_for_ball_moving_right)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::movingBall)
      {
        goto search_for_ball;
      }
      if (action_done)
      {
        goto search_for_ball_moving_left;
      }
    }
    action
    {
      GoToFieldCoordinates(Pose2f(theKeeper.optPosition.rotation + 45, theKeeper.optPosition.translation), 100, 100, 100, 10_deg, false, true);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = -45_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }

  state(search_for_ball_supporter)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::supporter)
      {
        goto search_for_ball;
      }
    }
    action
    {
      GoToFieldCoordinates(theKeeper.optPosition, 100, 100, 100, 10_deg, false, true);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = -45_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }

  state(search_for_ball_penalty)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::penalty)
      {
        goto search_for_ball;
      }
    }
    action
    {
      //TODO Add search behavior, that's not stupid
      Walk(WalkRequest::speed, 0, 0, -60_deg);
      theHeadControlRequest.controlType = HeadControlRequest::direct;
      theHeadControlRequest.pan = -45_deg;
      theHeadControlRequest.tilt = 30_deg;
    }
  }

  state(search_for_ball_symmetry)
  {
    transition
    {
      if (theKeeper.ballSearchState != Keeper::KeeperBallSearchState::locate)
      {
        goto search_for_ball;
      }
      if (action_done && state_time > 2000)
      {
        //maybe rotate different?
        goto search_for_ball;
      }
    }
    action
    {
      GoToFieldCoordinates(Pose2f(theKeeper.optPosition.rotation + 180, theKeeper.optPosition.translation), 100, 100, 100, 10_deg, false, true);
    }
  }
}