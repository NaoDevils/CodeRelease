/**
 * @file MocapCalibration.h
 * Declaration of a struct for representing the calibration values for the mocap system.
 * @author Janine Hemmers
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(MocapCalibration,
{,
  (int)(-1) rigidbodyID,
});