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
  gyroXBuffer.fill(0_deg);
  gyroYBuffer.fill(0_deg);
}

void GyroFallDownStateDetector::update(FallDownState& fallDownState)
{
  Angle angleXZ = useIMUModel ? theIMUModel.orientation.y() : theInertialSensorData.angle.y();
  Angle angleYZ = useIMUModel ? theIMUModel.orientation.x() : theInertialSensorData.angle.x();
  
  if (theFrameInfo.time - fallDownTime >= fallenFinishedTime)
  {
    if (angleXZ > uprightAngleThreshold || (angleXZ > 30_deg && angleYZ > 30_deg) || (angleXZ > 30_deg && angleYZ < -30_deg))
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::front;
    }
    else if(angleXZ < -uprightAngleThreshold || (angleXZ < -30_deg && angleYZ > 30_deg) || (angleXZ < -30_deg && angleYZ < -30_deg))
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::back;
    }
    else if (angleYZ < -uprightAngleThreshold)
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::left;
    }
    else if(angleYZ > uprightAngleThreshold)
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::right;
    }
    else
    {
      if (uprightAfterSpecialAction)
      {
        if (!(theMotionInfo.motion == MotionRequest::specialAction
          && theMotionInfo.specialActionRequest.specialAction >= SpecialActionRequest::standUpBackNao
          && theMotionInfo.specialActionRequest.specialAction < SpecialActionRequest::numOfSpecialActionIDs))
          fallDownState.state = FallDownState::upright;
        else
          fallDownState.state = FallDownState::standingUp;
      }
      else
        fallDownState.state = FallDownState::upright;
    }
  }

  // if gyro is indicating fall, do not set state to onGround, but to falling
  if (useGyroSpeed && fallDownState.state != FallDownState::upright)
  {
    gyroXBuffer.push_front(theInertialSensorData.gyro.x());
    gyroYBuffer.push_front(theInertialSensorData.gyro.y());
    if (std::abs(gyroXBuffer.sum()) < maxGyroForStandup || std::abs(gyroYBuffer.sum()) < maxGyroForStandup)
      fallDownState.state = FallDownState::falling;
  }

  if(fallDownState.state == FallDownState::upright)
  {
    //Set state:

    // state falling if average angle of robot is too high and robot is not trying to get up
    if(std::abs(angleXZ) > 35_deg || std::abs(angleYZ) > 35_deg)
    {
      if (!(theMotionInfo.motion == MotionRequest::specialAction
         && theMotionInfo.specialActionRequest.specialAction >= SpecialActionRequest::standUpBackNao
           && theMotionInfo.specialActionRequest.specialAction < SpecialActionRequest::numOfSpecialActionIDs))
      {
        fallDownState.state = FallDownState::falling;
        // This is for head protection. Only front or back decision matters
        if (angleXZ > 30_deg) // was 0.3f before 2019
          fallDownState.direction = FallDownState::front;
        else if (angleXZ < -30_deg) // was 0.3f before 2019
          fallDownState.direction = FallDownState::back;
        else
          fallDownState.direction = FallDownState::none;
        fallDownTime = theFrameInfo.time;
      }
    }
  }
}


MAKE_MODULE(GyroFallDownStateDetector, sensing)
