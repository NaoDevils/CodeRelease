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
  (Vector3f)(Vector3f::Zero()) comOffset,
  (std::array<Vector2a, JoinedIMUData::numOfInertialDataSources>) imuAngleOffsets,
  (float)(9.81f) gravity,
  (bool)(false) deactivateSensorControl,
  (bool)(false) bodyAngleCalibrated,
  (bool)(false) walkCalibrated
);
