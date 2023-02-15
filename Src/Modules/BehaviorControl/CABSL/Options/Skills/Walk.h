/**
* Walk Skill
*/
option(Walk, ((WalkRequest) RequestType) requestType, (float) x, (float) y, (Angle) rot, ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) stepRequest, ((WalkRequest) RotationType)(WalkRequest::RotationType::irrelevant) rotationType)
{
  initial_state(walk)
  {
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.requestType = requestType;
    theMotionRequest.walkRequest.request.translation.x() = x;
    theMotionRequest.walkRequest.request.translation.y() = y;
    theMotionRequest.walkRequest.request.rotation = rot;
    theMotionRequest.walkRequest.stepRequest = stepRequest;
    theMotionRequest.walkRequest.rotationType = rotationType;
    theMotionRequest.walkRequest.accLimits = Pose2f(); // default : set to zero -> no changes
  }
}
