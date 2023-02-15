/**
 * @file JoinedIMUDataProvider.h
 *
 * 
 *
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(JoinedIMUDataProvider,
  REQUIRES(InertialSensorData),
  REQUIRES(InertialData),
  REQUIRES(IMUModel),
  REQUIRES(WalkingEngineParams),
  USES(WalkCalibration),
  PROVIDES(JoinedIMUData)
);

class JoinedIMUDataProvider : public JoinedIMUDataProviderBase
{
private:
  /** Executes this module
   * @param joinedIMUData The data structure that is filled by this module
   */
  void update(JoinedIMUData& joinedIMUData);

  void drawPlots(JoinedIMUData& joinedIMUData);

  RingBuffer<Vector2a, MAX_DELAY_FRAMES> imuAngleBuffer;
};
