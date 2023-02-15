#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Joints.h"
#include "Tools/SensorData.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE_WITH_BASE(JointSensorData, JointAngles,

  JointSensorData();
  void draw() const;
  ,
  (std::array<short, Joints::numOfJoints>) currents, /**< The currents of all motors. */
  (std::array<unsigned char, Joints::numOfJoints>) temperatures, /**< The temperature of all motors. */
  (std::array<unsigned char, Joints::numOfJoints>) status /**< The temperature status of all motors. */
);