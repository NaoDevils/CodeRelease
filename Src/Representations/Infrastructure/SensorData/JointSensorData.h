#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Joints.h"
#include "Tools/SensorData.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE_WITH_BASE(JointSensorData, JointAngles,
{
  JointSensorData();
  ,
  (std::array<short, Joints::numOfJoints>) currents, /**< The currents of all motors. */
  (std::array<unsigned char, Joints::numOfJoints>) temperatures, /**< The temperature of all motors. */
  (std::array<unsigned char, Joints::numOfJoints>) status, /**< The temperature status of all motors. */
});

inline JointSensorData::JointSensorData() :
  JointAngles()
{
  currents.fill(SensorData::off);
  temperatures.fill(0);
  status.fill(0);
}