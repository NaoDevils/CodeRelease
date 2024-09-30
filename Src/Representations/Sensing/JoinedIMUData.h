#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
 * Encapsulates the IMU sensor data from Lola and different other filtered IMU models.
 */
STREAMABLE(JoinedIMUData,
  void draw() const;
  ENUM(InertialDataSource,
    inertialSensorData,
    inertialData,
    imuModel
  ),
  (std::array<InertialSensorData, JoinedIMUData::numOfInertialDataSources>) imuData
);
