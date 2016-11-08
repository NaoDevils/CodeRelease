/** go to target on field, takes rotation in degree as always */
#include <algorithm>
option(GoToFieldCoordinates,
  (Pose2f) optPosition, 
  (float) thresh_x_front, 
  (float) thresh_x_back, 
  (float) thresh_y, 
  (float) thresh_rot, 
  (bool) stop_at_target,
  (bool) previewArrival)
{
  CROSS("behavior:GoToFieldCoordinates:Thresholds",optPosition.translation.x(),optPosition.translation.y(),20,5,Drawings::solidPen,ColorRGBA::red);
  Pose2f destRel(Pose2f(Angle::fromDegrees(optPosition.rotation),optPosition.translation.x(),optPosition.translation.y())-theRobotPoseAfterPreview);
  Pose2f destRelWOPreview(Pose2f(Angle::fromDegrees(optPosition.rotation), optPosition.translation.x(), optPosition.translation.y()) - theRobotPose);
  Pose2f destToCheck = (previewArrival ? destRel : destRelWOPreview);
  initial_state(positioning)
  {
    transition
    {
      float thresh_x_front_mod = std::min(20.f,thresh_x_front);
      float thresh_x_back_mod = std::min(20.f,thresh_x_back);
      float thresh_y_mod = std::min(20.f,thresh_y);
      float thresh_rot_mod = std::min(5.f,thresh_rot);
      if (destToCheck.translation.x() > -thresh_x_front_mod
        && destToCheck.translation.x() < thresh_x_back_mod
        && std::abs(destToCheck.translation.y()) < thresh_y_mod
        && std::abs(toDegrees(destToCheck.rotation)) < thresh_rot_mod)
        goto arrived;
    }
    
    action
    {
      Walk(WalkRequest::destination, destRel.translation.x(), destRel.translation.y(), toDegrees(destRel.rotation));
    }
  }

  target_state(arrived)
  {
    transition
    {
      float alpha = std::min(((float)state_time)/3000,1.f);
      float thresh_x_front_mod = std::min(20.f, thresh_x_front)*(1-alpha)
        + std::max(20.f,thresh_x_front)*alpha;
      float thresh_x_back_mod = std::min(20.f, thresh_x_back)*(1-alpha)
        + std::max(20.f,thresh_x_back)*alpha;
      float thresh_y_mod = std::min(20.f, thresh_y)*(1-alpha)
        + std::max(20.f,thresh_y)*alpha;
      float thresh_rot_mod = std::min(5.f, thresh_rot)*(1-alpha)
        + std::max(5.f,thresh_rot)*alpha;
      ELLIPSE("behavior:GoToFieldCoordinates:Thresholds",
        Vector2f(optPosition.translation.x()+(thresh_x_front_mod-thresh_x_back_mod)/2,optPosition.translation.y()),
        (thresh_x_front_mod+thresh_x_back_mod)/2,
        thresh_y_mod,
        0, 10, Drawings::solidPen,
        ColorRGBA::red,
        Drawings::noBrush,
        ColorRGBA::red);

      if (destToCheck.translation.x() < -thresh_x_front_mod - 10 || destToCheck.translation.x() > thresh_x_back_mod + 10
        || std::abs(destToCheck.translation.y()) > thresh_y_mod + 10 || std::abs(toDegrees(destToCheck.rotation)) > thresh_rot_mod + 5)
        goto positioning;
    }
    
    action
    {
      if (stop_at_target)
        Walk(WalkRequest::speed, 0,0,0);
      else
      {
        Walk(WalkRequest::destination,destRel.translation.x(), destRel.translation.y(), toDegrees(destRel.rotation));
      }
    }
  }
}