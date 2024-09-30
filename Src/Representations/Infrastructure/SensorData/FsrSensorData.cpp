#include "FsrSensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

FsrSensorData::FsrSensorData()
{
  left.fill(SensorData::off);
  right.fill(SensorData::off);
}

void FsrSensorData::draw()
{
  PLOT("representation:FsrSensorData:leftTotal", leftTotal);
  PLOT("representation:FsrSensorData:rightTotal", rightTotal);
  PLOT("representation:FsrSensorData:supportFoot", calcSupportFoot() / 10.f);

  PLOT("representation:FsrSensorData:left:fl", left[fl]);
  PLOT("representation:FsrSensorData:left:fr", left[fr]);
  PLOT("representation:FsrSensorData:left:bl", left[bl]);
  PLOT("representation:FsrSensorData:left:br", left[br]);

  PLOT("representation:FsrSensorData:right:fl", right[fl]);
  PLOT("representation:FsrSensorData:right:fr", right[fr]);
  PLOT("representation:FsrSensorData:right:bl", right[bl]);
  PLOT("representation:FsrSensorData:right:br", right[br]);

#ifdef LOGGING
  DEBUG_RESPONSE("representation:FsrSensorData:CSVLog")
  {
    LOG("FsrSensorData", "left_fl", left[fl]);
    LOG("FsrSensorData", "left_fr", left[fr]);
    LOG("FsrSensorData", "left_bl", left[bl]);
    LOG("FsrSensorData", "left_br", left[br]);

    LOG("FsrSensorData", "right_fl", right[fl]);
    LOG("FsrSensorData", "right_fr", right[fr]);
    LOG("FsrSensorData", "right_bl", right[bl]);
    LOG("FsrSensorData", "right_br", right[br]);
  }
#endif
}

float FsrSensorData::calcSupportFoot() const
{
  // from HTWKWalk
  // positive is left foot
  static constexpr float max_pressure = 5.0f;
  static const std::array<float, FsrSensorData::numOfFsrSensorPositions* 2> weights = {0.8f, 0.3f, 0.8f, 0.3f, -0.3f, -0.8f, -0.3f, -0.8f};
  static std::array<float, FsrSensorData::numOfFsrSensorPositions* 2> max_left = {0.f, 0.f, 0.f, 0.f};
  static std::array<float, FsrSensorData::numOfFsrSensorPositions* 2> max_right = {0.f, 0.f, 0.f, 0.f};
  float total = 0;
  float weighted = 0;
  for (int i = 0; i < 4; i++)
  {
    float value = std::min(max_pressure, left[i]);
    max_left[i] = std::max(max_left[i], value);
    float weight = weights[i];
    if (max_left[i] > 0.f)
    {
      value /= max_left[i];
      total += value;
      weighted += weight * value;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    float value = std::min(max_pressure, right[i]);
    max_right[i] = std::max(max_right[i], value);
    float weight = weights[i + 4];
    if (max_right[i] > 0.f)
    {
      value /= max_right[i];
      total += value;
      weighted += weight * value;
    }
  }

  if (total == 0)
  {
    return 0;
  }
  return (weighted / total);
}
