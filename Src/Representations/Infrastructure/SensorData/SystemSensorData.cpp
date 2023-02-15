#include "SystemSensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void SystemSensorData::draw() const
{
  //DECLARE_PLOT("representation:SystemSensorData:cpuTemperature");
  //DECLARE_PLOT("representation:SystemSensorData:batteryCurrent");
  //DECLARE_PLOT("representation:SystemSensorData:batteryLevel");
  //DECLARE_PLOT("representation:SystemSensorData:batteryTemperature");

  PLOT("representation:SystemSensorData:cpuTemperature", cpuTemperature);
  PLOT("representation:SystemSensorData:batteryCurrent", batteryCurrent);
  PLOT("representation:SystemSensorData:batteryLevel", batteryLevel);
  PLOT("representation:SystemSensorData:batteryTemperature", batteryTemperature);
}
