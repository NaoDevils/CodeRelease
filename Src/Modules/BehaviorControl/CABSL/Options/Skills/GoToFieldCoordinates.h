/** go to target on field */
#include <algorithm>
option(GoToFieldCoordinates,
  (Pose2f) optPosition, 
  (float) thresh_x_front, 
  (float) thresh_x_back, 
  (float) thresh_y, 
  (Angle) thresh_rot, 
  (bool) stop_at_target,
  (bool) previewArrival,
  (bool)(true) preciseArrival,
  ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) stepRequest,
  ((WalkRequest) RotationType)(WalkRequest::RotationType::irrelevant) rotationType)
{
  CROSS("behavior:GoToFieldCoordinates:Thresholds", optPosition.translation.x(), optPosition.translation.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);
  Pose2f destRel(optPosition - theRobotPoseAfterPreview);
  Pose2f destRelWOPreview(optPosition - theRobotPose);
  Pose2f destToCheck = (previewArrival ? destRel : destRelWOPreview);
  initial_state(positioning)
  {
    gotoFieldCoordinatesFinished = false;
    transition
    {
      float thresh_x_front_mod = thresh_x_front;
      float thresh_x_back_mod = thresh_x_back;
      float thresh_y_mod = thresh_y;
      Angle thresh_rot_mod = thresh_rot;

      if (preciseArrival)
      {
        thresh_x_front_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_x_front_mod);
        thresh_x_back_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_x_back_mod);
        thresh_y_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_y_mod);
        thresh_rot_mod = std::min(theBehaviorConfiguration.gotoBaseThreshRot, thresh_rot_mod);
      }

      if (destToCheck.translation.x() > -thresh_x_front_mod && destToCheck.translation.x() < thresh_x_back_mod && std::abs(destToCheck.translation.y()) < thresh_y_mod
          && std::abs(destToCheck.rotation) < thresh_rot_mod)
        goto arrived;
    }

    action
    {
      Walk(WalkRequest::destination, destRel.translation.x(), destRel.translation.y(), destRel.rotation, stepRequest, rotationType);
    }
  }

  target_state(arrived)
  {
    gotoFieldCoordinatesFinished = true;
    transition
    {
      float alpha = std::min(((float)state_time) / theBehaviorConfiguration.gotoThreshMaxTime, 1.f);
      float thresh_x_front_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_x_front) * (1 - alpha) + std::max(theBehaviorConfiguration.gotoBaseThresh, thresh_x_front) * alpha;
      float thresh_x_back_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_x_back) * (1 - alpha) + std::max(theBehaviorConfiguration.gotoBaseThresh, thresh_x_back) * alpha;
      float thresh_y_mod = std::min(theBehaviorConfiguration.gotoBaseThresh, thresh_y) * (1 - alpha) + std::max(theBehaviorConfiguration.gotoBaseThresh, thresh_y) * alpha;
      Angle thresh_rot_mod = std::min(theBehaviorConfiguration.gotoBaseThreshRot, thresh_rot) * (1 - alpha) + std::max(theBehaviorConfiguration.gotoBaseThreshRot, thresh_rot) * alpha;
      ELLIPSE("behavior:GoToFieldCoordinates:Thresholds",
          Vector2f(optPosition.translation.x() + (thresh_x_front_mod - thresh_x_back_mod) / 2, optPosition.translation.y()),
          (thresh_x_front_mod + thresh_x_back_mod) / 2,
          thresh_y_mod,
          0,
          10,
          Drawings::solidPen,
          ColorRGBA::red,
          Drawings::noBrush,
          ColorRGBA::red);

      if (destToCheck.translation.x() < -thresh_x_front_mod - 10 || destToCheck.translation.x() > thresh_x_back_mod + 10
          || std::abs(destToCheck.translation.y()) > thresh_y_mod + 10 || std::abs(destToCheck.rotation) > thresh_rot_mod + 5_deg)
        goto positioning;
    }

    action
    {
      if (stop_at_target)
      {
        if (state_time > 3000 && ((theRoleSymbols.role != BehaviorData::keeper && theRoleSymbols.role != BehaviorData::replacementKeeper) || theGameInfo.state != STATE_PLAYING)
            && ((theBallSymbols.timeSinceLastSeenByTeam < 8000.f && theBallSymbols.ballPositionRelative.norm() > 1300.f) || theGameInfo.state != STATE_PLAYING))
          Stand();
        else
          Walk(WalkRequest::speed, 0, 0, 0);
      }
      else
      {
        Walk(WalkRequest::destination, destRel.translation.x(), destRel.translation.y(), destRel.rotation, stepRequest, rotationType);
      }
    }
  }
}
