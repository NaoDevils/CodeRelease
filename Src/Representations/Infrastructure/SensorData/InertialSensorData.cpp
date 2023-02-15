#include "InertialSensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

void InertialSensorData::draw() const
{
  PLOT("representation:InertialSensorData:gyro:x", gyro.x().toDegrees());
  PLOT("representation:InertialSensorData:gyro:y", gyro.y().toDegrees());
  PLOT("representation:InertialSensorData:gyro:z", gyro.z().toDegrees());

  PLOT("representation:InertialSensorData:acc:x", acc.x());
  PLOT("representation:InertialSensorData:acc:y", acc.y());
  PLOT("representation:InertialSensorData:acc:z", acc.z());

  PLOT("representation:InertialSensorData:angle:x", angle.x().toDegrees());
  PLOT("representation:InertialSensorData:angle:y", angle.y().toDegrees());

#ifdef LOGGING
  DEBUG_RESPONSE("representation:InertialSensorData:CSVLog")
  {
    LOG("InertialSensorData", "gyro_x", gyro.x());
    LOG("InertialSensorData", "gyro_y", gyro.y());
    LOG("InertialSensorData", "gyro_z", gyro.z());

    LOG("InertialSensorData", "acc_x", acc.x());
    LOG("InertialSensorData", "acc_y", acc.y());
    LOG("InertialSensorData", "acc_z", acc.z());

    LOG("InertialSensorData", "angle_x", angle.x());
    LOG("InertialSensorData", "angle_y", angle.y());
  }
#endif
}
