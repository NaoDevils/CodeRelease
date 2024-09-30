/**
* @file WalkCalibration.h
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Tools/Streams/AutoStreamable.h"

/**
* @class WalkCalibration 
* 
*/
STREAMABLE(WalkCalibration,,
  (Angle[12]) legJointCalibration,
  (std::array<Vector2a, JoinedIMUData::numOfInertialDataSources>) imuAngleOffsets,
  (float)(9.81f) gravity,
  (Vector2a)(Vector2a::Zero()) fieldInclination,
  (float)(1.f) qualityOfRobotHardware,
  (bool)(false) deactivateSensorControl,
  (bool)(false) bodyAngleCalibrated,
  (float)(0.f) bodyAngleProgress,
  (bool)(false) walkCalibrated
);
