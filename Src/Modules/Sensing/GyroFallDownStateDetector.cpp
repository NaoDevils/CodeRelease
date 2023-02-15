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
#include "Tools/Debugging/Annotation.h"

//#include "Tools/Debugging/CSVLogger.h"

GyroFallDownStateDetector::GyroFallDownStateDetector()
{
  fallDownTime = 0;
  fallenFinishedTime = 1000;
  lastFallDownState = FallDownState::undefined;
  gyroXBuffer.fill(0_deg);
  gyroYBuffer.fill(0_deg);
}

void GyroFallDownStateDetector::update(FallDownState& fallDownState)
{
  DECLARE_PLOT("module:GyroFallDownStateDetector:GyroBufferSum:X");
  DECLARE_PLOT("module:GyroFallDownStateDetector:GyroBufferSum:Y");
  DECLARE_PLOT("module:GyroFallDownStateDetector:GyroBufferSum:Threshold");
  Angle angleXZ = theJoinedIMUData.imuData[anglesource].angle.y();
  Angle angleYZ = theJoinedIMUData.imuData[anglesource].angle.x();
  float fsrSum = theFsrSensorData.leftTotal + theFsrSensorData.rightTotal;
  if (fsrMin > fsrSum)
    fsrMin = fsrSum;
  if (fsrMax < fsrSum)
    fsrMax = fsrSum;

  if (theFrameInfo.time - fallDownTime >= fallenFinishedTime)
  {
    if ((angleXZ > uprightAngleThreshold || (angleXZ > 30_deg && angleYZ > 30_deg) || (angleXZ > 30_deg && angleYZ < -30_deg))) // && theRobotInfo.penalty == PENALTY_NONE
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::front;
    }
    else if ((angleXZ < -uprightAngleThreshold || (angleXZ < -30_deg && angleYZ > 30_deg) || (angleXZ < -30_deg && angleYZ < -30_deg))) // && theRobotInfo.penalty == PENALTY_NONE
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::back;
    }
    else if (angleYZ < -uprightAngleThreshold) // && theRobotInfo.penalty == PENALTY_NONE
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::left;
    }
    else if (angleYZ > uprightAngleThreshold) // && theRobotInfo.penalty == PENALTY_NONE
    {
      fallDownState.state = FallDownState::onGround;
      fallDownState.direction = FallDownState::right;
    }
    else
    {
      if (uprightAfterSpecialAction)
      {
        if (!theMotionInfo.inBlockMotion() && !theMotionInfo.inStandUpMotion())
          fallDownState.state = FallDownState::upright;
        else
          fallDownState.state = FallDownState::standingUp;
      }
      else
        fallDownState.state = FallDownState::upright;
    }
  }

  gyroXBuffer.push_front(abs(theJoinedIMUData.imuData[anglesource].gyro.x()));
  gyroYBuffer.push_front(abs(theJoinedIMUData.imuData[anglesource].gyro.y()));

  // if gyro is indicating fall, do not set state to onGround, but to falling
  if (useGyroSpeed && fallDownState.state != FallDownState::upright && !theMotionInfo.inBlockMotion() && !theMotionInfo.inStandUpMotion() && (theFrameInfo.time - fallDownTime < 10 * fallenFinishedTime))
  {
    PLOT("module:GyroFallDownStateDetector:GyroBufferSum:X", abs(gyroXBuffer.sum().toDegrees()));
    PLOT("module:GyroFallDownStateDetector:GyroBufferSum:Y", abs(gyroYBuffer.sum().toDegrees()));
    PLOT("module:GyroFallDownStateDetector:GyroBufferSum:Threshold", maxGyroForStandup.toDegrees());

    if ((std::abs(gyroXBuffer.sum()) > maxGyroForStandup || std::abs(gyroYBuffer.sum()) > maxGyroForStandup) && (fsrSum - fsrMin) >= 1.f)
    {
      fallDownState.state = FallDownState::falling;
    }
  }


  if (fallDownState.state == FallDownState::upright)
  {
    //Set state:

    // state falling if average angle of robot is too high and robot is not trying to get up
    if (std::abs(angleXZ) > fallDownAngle.y() || std::abs(angleYZ) > fallDownAngle.x()) // old value 35_deg
    {
      if (!theMotionInfo.inBlockMotion() && !theMotionInfo.inStandUpMotion())
      {
        if ((fsrSum - fsrMin) < 1.f)
        {
        } //  check if no ground contact
        else
        {
          fallDownState.state = FallDownState::falling;
          // This is for head protection. Only front or back decision matters
          if (angleXZ > fallDownAngle.y()) // was 0.3f before 2019
            fallDownState.direction = FallDownState::front;
          else if (angleXZ < -fallDownAngle.y()) // was 0.3f before 2019
            fallDownState.direction = FallDownState::back;
          else
            fallDownState.direction = FallDownState::none;
          fallDownTime = theFrameInfo.time;
        }
      }
    }
  }

  if (fallDownState.state == FallDownState::falling && lastFallDownState != fallDownState.state)
    ANNOTATION("FallDownState", "Robot is falling!");
  lastFallDownState = fallDownState.state;
}


MAKE_MODULE(GyroFallDownStateDetector, sensing)
