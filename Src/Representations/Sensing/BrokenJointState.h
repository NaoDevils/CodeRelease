/**
 * @file Representations/Sensing/BrokenJointState.h
 *
 * Declaration of struct BrokenJointState
 *
 * @author Diana Kleingarn
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"

/**
 * @struct BrokenJointState
 *
 * A struct that represents the current state of the robot's body
 */
STREAMABLE(BrokenJointState,
  ENUM(JointState,
    undefined,
    alright,
    malfunction
  );

 
  ,
  (JointState)(undefined) jointState, /**< Current state of the robot's body. */
  (std::array<bool,Joints::numOfJoints>) brokenJointStatus, // shows possibly broken joints
  (std::array<bool,Joints::numOfJoints>) stuckJointStatus // shows possibly stuck joints
);
