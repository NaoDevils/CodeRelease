#include "JointSensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

JointSensorData::JointSensorData() : JointAngles()
{
  currents.fill(static_cast<short>(SensorData::off));
  temperatures.fill(0);
  status.fill(0);
}

void JointSensorData::draw() const
{
  PLOT("representation:JointSensorData:headYaw:angle", angles[Joints::headYaw].toDegrees());
  PLOT("representation:JointSensorData:headYaw:current", currents[Joints::headYaw]);
  PLOT("representation:JointSensorData:headYaw:temperature", temperatures[Joints::headYaw]);
  PLOT("representation:JointSensorData:headYaw:status", status[Joints::headYaw]);

  PLOT("representation:JointSensorData:headPitch:angle", angles[Joints::headPitch].toDegrees());
  PLOT("representation:JointSensorData:headPitch:current", currents[Joints::headPitch]);
  PLOT("representation:JointSensorData:headPitch:temperature", temperatures[Joints::headPitch]);
  PLOT("representation:JointSensorData:headPitch:status", status[Joints::headPitch]);

  PLOT("representation:JointSensorData:lShoulderPitch:angle", angles[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointSensorData:lShoulderPitch:current", currents[Joints::lShoulderPitch]);
  PLOT("representation:JointSensorData:lShoulderPitch:temperature", temperatures[Joints::lShoulderPitch]);
  PLOT("representation:JointSensorData:lShoulderPitch:status", status[Joints::lShoulderPitch]);

  PLOT("representation:JointSensorData:lShoulderRoll:angle", angles[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointSensorData:lShoulderRoll:current", currents[Joints::lShoulderRoll]);
  PLOT("representation:JointSensorData:lShoulderRoll:temperature", temperatures[Joints::lShoulderRoll]);
  PLOT("representation:JointSensorData:lShoulderRoll:status", status[Joints::lShoulderRoll]);

  PLOT("representation:JointSensorData:lElbowYaw:angle", angles[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointSensorData:lElbowYaw:current", currents[Joints::lElbowYaw]);
  PLOT("representation:JointSensorData:lElbowYaw:temperature", temperatures[Joints::lElbowYaw]);
  PLOT("representation:JointSensorData:lElbowYaw:status", status[Joints::lElbowYaw]);

  PLOT("representation:JointSensorData:lElbowRoll:angle", angles[Joints::lElbowRoll].toDegrees());
  PLOT("representation:JointSensorData:lElbowRoll:current", currents[Joints::lElbowRoll]);
  PLOT("representation:JointSensorData:lElbowRoll:temperature", temperatures[Joints::lElbowRoll]);
  PLOT("representation:JointSensorData:lElbowRoll:status", status[Joints::lElbowRoll]);

  PLOT("representation:JointSensorData:lWristYaw:angle", angles[Joints::lWristYaw].toDegrees());
  PLOT("representation:JointSensorData:lWristYaw:current", currents[Joints::lWristYaw]);
  PLOT("representation:JointSensorData:lWristYaw:temperature", temperatures[Joints::lWristYaw]);
  PLOT("representation:JointSensorData:lWristYaw:status", status[Joints::lWristYaw]);

  PLOT("representation:JointSensorData:lHand:angle", angles[Joints::lHand].toDegrees());
  PLOT("representation:JointSensorData:lHand:current", currents[Joints::lHand]);
  PLOT("representation:JointSensorData:lHand:temperature", temperatures[Joints::lHand]);
  PLOT("representation:JointSensorData:lHand:status", status[Joints::lHand]);

  PLOT("representation:JointSensorData:rShoulderPitch:angle", angles[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointSensorData:rShoulderPitch:current", currents[Joints::rShoulderPitch]);
  PLOT("representation:JointSensorData:rShoulderPitch:temperature", temperatures[Joints::rShoulderPitch]);
  PLOT("representation:JointSensorData:rShoulderPitch:status", status[Joints::rShoulderPitch]);

  PLOT("representation:JointSensorData:rShoulderRoll:angle", angles[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointSensorData:rShoulderRoll:current", currents[Joints::rShoulderRoll]);
  PLOT("representation:JointSensorData:rShoulderRoll:temperature", temperatures[Joints::rShoulderRoll]);
  PLOT("representation:JointSensorData:rShoulderRoll:status", status[Joints::rShoulderRoll]);

  PLOT("representation:JointSensorData:rElbowYaw:angle", angles[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointSensorData:rElbowYaw:current", currents[Joints::rElbowYaw]);
  PLOT("representation:JointSensorData:rElbowYaw:temperature", temperatures[Joints::rElbowYaw]);
  PLOT("representation:JointSensorData:rElbowYaw:status", status[Joints::rElbowYaw]);

  PLOT("representation:JointSensorData:rElbowRoll:angle", angles[Joints::rElbowRoll].toDegrees());
  PLOT("representation:JointSensorData:rElbowRoll:current", currents[Joints::rElbowRoll]);
  PLOT("representation:JointSensorData:rElbowRoll:temperature", temperatures[Joints::rElbowRoll]);
  PLOT("representation:JointSensorData:rElbowRoll:status", status[Joints::rElbowRoll]);

  PLOT("representation:JointSensorData:rWristYaw:angle", angles[Joints::rWristYaw].toDegrees());
  PLOT("representation:JointSensorData:rWristYaw:current", currents[Joints::rWristYaw]);
  PLOT("representation:JointSensorData:rWristYaw:temperature", temperatures[Joints::rWristYaw]);
  PLOT("representation:JointSensorData:rWristYaw:status", status[Joints::rWristYaw]);

  PLOT("representation:JointSensorData:rHand:angle", angles[Joints::rHand].toDegrees());
  PLOT("representation:JointSensorData:rHand:current", currents[Joints::rHand]);
  PLOT("representation:JointSensorData:rHand:temperature", temperatures[Joints::rHand]);
  PLOT("representation:JointSensorData:rHand:status", status[Joints::rHand]);

  PLOT("representation:JointSensorData:lHipYawPitch:angle", angles[Joints::lHipYawPitch].toDegrees());
  PLOT("representation:JointSensorData:lHipYawPitch:current", currents[Joints::lHipYawPitch]);
  PLOT("representation:JointSensorData:lHipYawPitch:temperature", temperatures[Joints::lHipYawPitch]);
  PLOT("representation:JointSensorData:lHipYawPitch:status", status[Joints::lHipYawPitch]);

  PLOT("representation:JointSensorData:lHipRoll:angle", angles[Joints::lHipRoll].toDegrees());
  PLOT("representation:JointSensorData:lHipRoll:current", currents[Joints::lHipRoll]);
  PLOT("representation:JointSensorData:lHipRoll:temperature", temperatures[Joints::lHipRoll]);
  PLOT("representation:JointSensorData:lHipRoll:status", status[Joints::lHipRoll]);

  PLOT("representation:JointSensorData:lHipPitch:angle", angles[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointSensorData:lHipPitch:current", currents[Joints::lHipPitch]);
  PLOT("representation:JointSensorData:lHipPitch:temperature", temperatures[Joints::lHipPitch]);
  PLOT("representation:JointSensorData:lHipPitch:status", status[Joints::lHipPitch]);

  PLOT("representation:JointSensorData:lKneePitch:angle", angles[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointSensorData:lKneePitch:current", currents[Joints::lKneePitch]);
  PLOT("representation:JointSensorData:lKneePitch:temperature", temperatures[Joints::lKneePitch]);
  PLOT("representation:JointSensorData:lKneePitch:status", status[Joints::lKneePitch]);

  PLOT("representation:JointSensorData:lAnklePitch:angle", angles[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointSensorData:lAnklePitch:current", currents[Joints::lAnklePitch]);
  PLOT("representation:JointSensorData:lAnklePitch:temperature", temperatures[Joints::lAnklePitch]);
  PLOT("representation:JointSensorData:lAnklePitch:status", status[Joints::lAnklePitch]);

  PLOT("representation:JointSensorData:lAnkleRoll:angle", angles[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointSensorData:lAnkleRoll:current", currents[Joints::lAnkleRoll]);
  PLOT("representation:JointSensorData:lAnkleRoll:temperature", temperatures[Joints::lAnkleRoll]);
  PLOT("representation:JointSensorData:lAnkleRoll:status", status[Joints::lAnkleRoll]);

  PLOT("representation:JointSensorData:rHipYawPitch:angle", angles[Joints::rHipYawPitch].toDegrees());
  PLOT("representation:JointSensorData:rHipYawPitch:current", currents[Joints::rHipYawPitch]);
  PLOT("representation:JointSensorData:rHipYawPitch:temperature", temperatures[Joints::rHipYawPitch]);
  PLOT("representation:JointSensorData:rHipYawPitch:status", status[Joints::rHipYawPitch]);

  PLOT("representation:JointSensorData:rHipRoll:angle", angles[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointSensorData:rHipRoll:current", currents[Joints::rHipRoll]);
  PLOT("representation:JointSensorData:rHipRoll:temperature", temperatures[Joints::rHipRoll]);
  PLOT("representation:JointSensorData:rHipRoll:status", status[Joints::rHipRoll]);

  PLOT("representation:JointSensorData:rHipPitch:angle", angles[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointSensorData:rHipPitch:current", currents[Joints::rHipPitch]);
  PLOT("representation:JointSensorData:rHipPitch:temperature", temperatures[Joints::rHipPitch]);
  PLOT("representation:JointSensorData:rHipPitch:status", status[Joints::rHipPitch]);

  PLOT("representation:JointSensorData:rKneePitch:angle", angles[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointSensorData:rKneePitch:current", currents[Joints::rKneePitch]);
  PLOT("representation:JointSensorData:rKneePitch:temperature", temperatures[Joints::rKneePitch]);
  PLOT("representation:JointSensorData:rKneePitch:status", status[Joints::rKneePitch]);

  PLOT("representation:JointSensorData:rAnklePitch:angle", angles[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointSensorData:rAnklePitch:current", currents[Joints::rAnklePitch]);
  PLOT("representation:JointSensorData:rAnklePitch:temperature", temperatures[Joints::rAnklePitch]);
  PLOT("representation:JointSensorData:rAnklePitch:status", status[Joints::rAnklePitch]);

  PLOT("representation:JointSensorData:rAnkleRoll:angle", angles[Joints::rAnkleRoll].toDegrees());
  PLOT("representation:JointSensorData:rAnkleRoll:current", currents[Joints::rAnkleRoll]);
  PLOT("representation:JointSensorData:rAnkleRoll:temperature", temperatures[Joints::rAnkleRoll]);
  PLOT("representation:JointSensorData:rAnkleRoll:status", status[Joints::rAnkleRoll]);

#ifdef LOGGING
  DEBUG_RESPONSE("representation:JointSensorData:CSVLog")
  {
    LOG("JointSensorData", "headYaw", angles[Joints::headYaw]);
    LOG("JointSensorData", "headPitch", angles[Joints::headPitch]);

    LOG("JointSensorData", "lShoulderPitch", angles[Joints::lShoulderPitch]);
    LOG("JointSensorData", "lShoulderRoll", angles[Joints::lShoulderRoll]);
    LOG("JointSensorData", "lElbowYaw", angles[Joints::lElbowYaw]);
    LOG("JointSensorData", "lElbowRoll", angles[Joints::lElbowRoll]);
    LOG("JointSensorData", "lWristYaw", angles[Joints::lWristYaw]);
    LOG("JointSensorData", "lHand", angles[Joints::lHand]);

    LOG("JointSensorData", "rShoulderPitch", angles[Joints::rShoulderPitch]);
    LOG("JointSensorData", "rShoulderRoll", angles[Joints::rShoulderRoll]);
    LOG("JointSensorData", "rElbowYaw", angles[Joints::rElbowYaw]);
    LOG("JointSensorData", "rElbowRoll", angles[Joints::rElbowRoll]);
    LOG("JointSensorData", "rWristYaw", angles[Joints::rWristYaw]);
    LOG("JointSensorData", "rHand", angles[Joints::rHand]);

    LOG("JointSensorData", "lHipYawPitch", angles[Joints::lHipYawPitch]);
    LOG("JointSensorData", "lHipRoll", angles[Joints::lHipRoll]);
    LOG("JointSensorData", "lHipPitch", angles[Joints::lHipPitch]);
    LOG("JointSensorData", "lKneePitch", angles[Joints::lKneePitch]);
    LOG("JointSensorData", "lAnklePitch", angles[Joints::lAnklePitch]);
    LOG("JointSensorData", "lAnkleRoll", angles[Joints::lAnkleRoll]);

    LOG("JointSensorData", "rHipYawPitch", angles[Joints::rHipYawPitch]);
    LOG("JointSensorData", "rHipRoll", angles[Joints::rHipRoll]);
    LOG("JointSensorData", "rHipPitch", angles[Joints::rHipPitch]);
    LOG("JointSensorData", "rKneePitch", angles[Joints::rKneePitch]);
    LOG("JointSensorData", "rAnklePitch", angles[Joints::rAnklePitch]);
    LOG("JointSensorData", "rAnkleRoll", angles[Joints::rAnkleRoll]);
  }
#endif
}
