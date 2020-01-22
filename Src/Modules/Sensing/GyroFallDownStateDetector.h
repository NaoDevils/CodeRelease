/**
* @file GyroFallDownStateDetector.h
*
* This file declares a module that provides information about the current state of the robot's body.
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"


MODULE(GyroFallDownStateDetector,
{ ,
  REQUIRES(InertialSensorData),
  REQUIRES(IMUModel),
  USES(MotionInfo),
  REQUIRES(FrameInfo),
  PROVIDES(FallDownState),
  DEFINES_PARAMETERS(
  {,
    (bool)(true) uprightAfterSpecialAction,
    (bool)(false) useIMUModel,
    (bool)(false) useGyroSpeed,
    (Angle)(5_deg) maxGyroForStandup,
    (Angle)(60_deg) uprightAngleThreshold,
  }),
});


/**
* @class GyroFallDownStateDetector
*
* A module for computing the current body state from sensor data
*/
class GyroFallDownStateDetector: public GyroFallDownStateDetectorBase
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

  RingBufferWithSum<Angle, 50> gyroXBuffer;
  RingBufferWithSum<Angle, 50> gyroYBuffer;

};
