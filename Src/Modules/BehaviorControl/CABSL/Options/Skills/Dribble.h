option(Dribble, (Pose2f) optPosition)
{
  timeStampLastDribbleExecution = theFrameInfo.time;
  libTactic.timeStampLastDribbleExecution = timeStampLastDribbleExecution;
  initial_state(dribble)
  {
    transition {}
    action
    {
      float ySpeed = theBallchaser.dribbleSpeedY;
      float xSpeed = theBallchaser.dribbleSpeedX;
      Angle rotSpeed = std::max<Angle>(std::min<Angle>(25_deg, Angle::normalize(optPosition.rotation - theRobotPoseAfterPreview.rotation)), -25_deg);
      Walk(WalkRequest::speed, xSpeed, ySpeed, rotSpeed);
    }
  }
}