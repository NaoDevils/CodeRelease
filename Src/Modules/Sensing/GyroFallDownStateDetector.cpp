/**
* @file GyroFallDownStateDetector.cpp
*
* This file implements a module that provides information about the current state of the robot's body.
*
* @author Oliver Urbann
*/

//#define LOGGING

#include "GyroFallDownStateDetector.h"
#include "Representations/Infrastructure/JointAngles.h"
//#include "Tools/Debugging/CSVLogger.h"

GyroFallDownStateDetector::GyroFallDownStateDetector()
{
  fallDownTime = 0;
  fallenFinishedTime = 1000;
}

void GyroFallDownStateDetector::update(FallDownState& fallDownState)
{
  float angleXZ(theInertialSensorData.angle.y());
  float angleYZ(theInertialSensorData.angle.x());
  float upRightAngleThreshold = 1.f;
  MODIFY("Module:GyroFallDownStateDetector:upRightAngleThreshold", upRightAngleThreshold);
  if (theFrameInfo.time - fallDownTime >= fallenFinishedTime)
  {
    if (angleXZ > upRightAngleThreshold || (angleXZ > 0.5f && angleYZ > 0.5f) || (angleXZ > 0.5f && angleYZ < -0.5f))
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::front;
    }
    else if(angleXZ < -upRightAngleThreshold || (angleXZ < -0.5f && angleYZ > 0.5f) || (angleXZ < -0.5f && angleYZ < -0.5f))
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::back;
    }
    else if (angleYZ < -upRightAngleThreshold)
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::left;
    }
    else if(angleYZ > upRightAngleThreshold)
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::right;
    }
    else
      fallDownState.state = FallDownState::upright;
  }

  if(fallDownState.state == FallDownState::upright)
  {
    //Set state:

    // state falling if average angle of robot is too high and robot is not trying to get up
    if(std::abs(angleXZ) > 0.6f || std::abs(angleYZ) > 0.6f)
    {
      if (!(theMotionInfo.motion == MotionRequest::specialAction
         && (theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpBackNao 
           || theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::standUpFrontNao)))
      {
        fallDownState.state = FallDownState::falling;
        fallDownTime = theFrameInfo.time;
      }
    }
  }

}


MAKE_MODULE(GyroFallDownStateDetector, sensing)
