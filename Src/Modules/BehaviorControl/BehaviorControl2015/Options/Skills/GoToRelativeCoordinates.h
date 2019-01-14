// rotation in degrees

option(GoToRelativeCoordinates, 
  (float) x, (float) y, (float) rot, 
  (float) thresh_x_front, (float) thresh_x_back, (float) thresh_y, (float) thresh_rot, 
  (bool) stop_at_target)
{
  
  initial_state(positioning)
  {
    transition
    {
      if (x > -thresh_x_front && x < thresh_x_back
        && std::abs(y) < thresh_y && std::abs(rot) < thresh_rot)
        goto arrived;
    }
    
    action
    {
      Walk(WalkRequest::destination, x, y, rot);
    }
  }

  target_state(arrived)
  {
    transition
    {
      if (x < -thresh_x_front-10 || x > thresh_x_back+10
        || std::abs(y) > thresh_y+10 || std::abs(rot) > thresh_rot+10)
        goto positioning;
    }

      action
    {
      if (stop_at_target)
        if (state_time > 3000)
          Stand();
        else
          Walk(WalkRequest::speed, 0, 0, 0);
      else
      {
        Walk(WalkRequest::destination, x, y, rot);
      }
    }
  }
}