/**
* @file GyroFallDownStateDetector.h
*
* This file declares a module that provides information about the current state of the robot's body.
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"


MODULE(GyroFallDownStateDetector,
  REQUIRES(JoinedIMUData),
  USES(MotionInfo),
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(RobotInfo),
  PROVIDES(FallDownState),
  LOADS_PARAMETERS(,
    (bool)(true) uprightAfterSpecialAction,
    (bool)(true) useGyroSpeed,
    (Vector2a)(Vector2a(20_deg,24_deg)) fallDownAngle,
    (Angle)(5_deg) maxGyroForStandup,
    (Angle)(60_deg) uprightAngleThreshold,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource
  )
);


/**
* @class GyroFallDownStateDetector
*
* A module for computing the current body state from sensor data
*/
class GyroFallDownStateDetector : public GyroFallDownStateDetectorBase
{
public:
  /** Default constructor */
  GyroFallDownStateDetector();

private:
  /** Executes this module
  * @param fallDownState The data structure that is filled by this module
  */
  void update(FallDownState& fallDownState);

  /**  */
  unsigned fallDownTime;

  unsigned fallenFinishedTime;

  unsigned lastFallDownState;

  RingBufferWithSum<Angle, 30> gyroXBuffer;
  RingBufferWithSum<Angle, 30> gyroYBuffer;
  float fsrMin = INFINITY;
  float fsrMax = -INFINITY;
};
