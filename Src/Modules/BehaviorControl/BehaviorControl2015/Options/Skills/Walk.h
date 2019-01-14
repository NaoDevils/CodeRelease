/**
* Walk Skill
*/
option(Walk, ((WalkRequest) RequestType) requestType, (float) x, (float) y, (float) rot, ((WalkRequest) StepRequest)(WalkRequest::StepRequest::none) stepRequest)
{
  initial_state(walk)
  {
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.requestType = requestType;
    theMotionRequest.walkRequest.request.translation.x() = x;
    theMotionRequest.walkRequest.request.translation.y() = y;
    theMotionRequest.walkRequest.request.rotation = Angle::fromDegrees(rot);
    theMotionRequest.walkRequest.stepRequest = stepRequest;
  }
}