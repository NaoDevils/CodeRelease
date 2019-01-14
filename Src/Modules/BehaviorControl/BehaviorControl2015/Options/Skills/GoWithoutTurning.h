option(GoWithoutTurning,
  (Pose2f) optPosition,
  (float) thresh_x_front,
  (float) thresh_x_back,
  (float) thresh_y,
  (Angle) thresh_rot,
  (bool) stop_at_target,
  (bool) previewArrival)
{
  initial_state(walk)
  {
    
    action
    {
      Vector2f toOptimalPosition = optPosition.translation - theRobotPose.translation;
      Vector2f targetPosition = theRobotPose.translation + toOptimalPosition.normalize(std::min(toOptimalPosition.norm(), 500.f));
      
      GoToFieldCoordinates(
        Pose2f(optPosition.rotation, targetPosition),
        thresh_x_front,
        thresh_x_back,
        thresh_y,
        thresh_rot,
        stop_at_target,
        previewArrival);
    }
  }
}