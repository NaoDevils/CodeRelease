/**
 * @file Representations/MotionControl/JointError.h
 * This file declares a struct that is used to represent Joint errors.
 * @author <A href="mailto:mrunal.hatwar@tu-dortmund.de">Mrunal Hatwar</A>
 */

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Joints.h"
#include "Tools/Streams/AutoStreamable.h"

#include <array>

STREAMABLE_WITH_BASE(JointError, JointAngles,

  JointError();
  void draw() const;
  ,
  (float)(1.f) qualityOfRobotHardware // 1.f means good robot, 0.f means bad robot // TODO right know it is the highest value
);
