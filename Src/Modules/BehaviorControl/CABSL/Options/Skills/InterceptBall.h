/*
* Walk sideways to intercept ball, do wide stance if @param useWideStance is true.
*/
option(InterceptBall, (bool) useWideStance)
{
  initial_state(intercept)
  {
    transition
    {
      // TODO -> wide stance state
    }
    action
    {
      Vector2f predictedBallRel = Transformation::fieldToRobot(theRobotPoseAfterPreview, theBallSymbols.ballPositionFieldPredicted);
      Walk(WalkRequest::speed, -theWalkingEngineParams.speedLimits.xBackward, (float)sgn(predictedBallRel.y()) * ((std::abs(predictedBallRel.y()) > 100) ? 150 : 50), 0);
      theMotionRequest.walkRequest.accLimits.translation.y() = theBehaviorConfiguration.behaviorParameters.ballInterceptionAccY;
    }
  }
}
