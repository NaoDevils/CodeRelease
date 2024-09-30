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

STREAMABLE(LegJoints,,
  (Angle)(0_deg) lHipYawPitch,
  (Angle)(0_deg) lHipRoll,
  (Angle)(0_deg) lHipPitch,
  (Angle)(0_deg) lKneePitch,
  (Angle)(0_deg) lAnklePitch,
  (Angle)(0_deg) lAnkleRoll,
  (Angle)(0_deg) rHipYawPitch,
  (Angle)(0_deg) rHipRoll,
  (Angle)(0_deg) rHipPitch,
  (Angle)(0_deg) rKneePitch,
  (Angle)(0_deg) rAnklePitch,
  (Angle)(0_deg) rAnkleRoll
);

STREAMABLE_WITH_BASE(JointError, JointAngles,

  JointError();
  void draw() const;
  ,
  (LegJoints) averageJointPlay,
  (float)(0.f) timeSpendWalking,
  (float)(1.f) qualityOfRobotHardware // 1.f means good robot, 0.f means bad robot // TODO right know it is the highest value
);
