option(HeadControl)
{

  initial_state(defaultMode)
  {
    transition {}
    action
    {
      theHeadControlRequest.controlType = HeadControlRequest::soccer;
    }
  }
}