/**
 * @file Representations/MotionControl/JointError.h
 * This file declares a struct that is used to represent Joint errors.
 * @author <A href="mailto:mrunal.hatwar@tu-dortmund.de">Mrunal Hatwar</A>
 */

#include "JointError.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

JointError::JointError() : JointAngles() {}

void JointError::draw() const
{
  PLOT("motionControl:JointError:headYaw:angle", angles[Joints::headYaw].toDegrees());

  PLOT("motionControl:JointError:headPitch:angle", angles[Joints::headPitch].toDegrees());

  PLOT("motionControl:JointError:lShoulderPitch:angle", angles[Joints::lShoulderPitch].toDegrees());

  PLOT("motionControl:JointError:lShoulderRoll:angle", angles[Joints::lShoulderRoll].toDegrees());

  PLOT("motionControl:JointError:lElbowYaw:angle", angles[Joints::lElbowYaw].toDegrees());

  PLOT("motionControl:JointError:lElbowRoll:angle", angles[Joints::lElbowRoll].toDegrees());

  PLOT("motionControl:JointError:lWristYaw:angle", angles[Joints::lWristYaw].toDegrees());

  PLOT("motionControl:JointError:lHand:angle", angles[Joints::lHand].toDegrees());

  PLOT("motionControl:JointError:rShoulderPitch:angle", angles[Joints::rShoulderPitch].toDegrees());

  PLOT("motionControl:JointError:rShoulderRoll:angle", angles[Joints::rShoulderRoll].toDegrees());

  PLOT("motionControl:JointError:rElbowYaw:angle", angles[Joints::rElbowYaw].toDegrees());

  PLOT("motionControl:JointError:rElbowRoll:angle", angles[Joints::rElbowRoll].toDegrees());

  PLOT("motionControl:JointError:rWristYaw:angle", angles[Joints::rWristYaw].toDegrees());

  PLOT("motionControl:JointError:rHand:angle", angles[Joints::rHand].toDegrees());

  PLOT("motionControl:JointError:lHipYawPitch:angle", angles[Joints::lHipYawPitch].toDegrees());

  PLOT("motionControl:JointError:lHipRoll:angle", angles[Joints::lHipRoll].toDegrees());

  PLOT("motionControl:JointError:lHipPitch:angle", angles[Joints::lHipPitch].toDegrees());

  PLOT("motionControl:JointError:lKneePitch:angle", angles[Joints::lKneePitch].toDegrees());

  PLOT("motionControl:JointError:lAnklePitch:angle", angles[Joints::lAnklePitch].toDegrees());

  PLOT("motionControl:JointError:lAnkleRoll:angle", angles[Joints::lAnkleRoll].toDegrees());

  PLOT("motionControl:JointError:rHipYawPitch:angle", angles[Joints::rHipYawPitch].toDegrees());

  PLOT("motionControl:JointError:rHipRoll:angle", angles[Joints::rHipRoll].toDegrees());

  PLOT("motionControl:JointError:rHipPitch:angle", angles[Joints::rHipPitch].toDegrees());

  PLOT("motionControl:JointError:rKneePitch:angle", angles[Joints::rKneePitch].toDegrees());

  PLOT("motionControl:JointError:rAnklePitch:angle", angles[Joints::rAnklePitch].toDegrees());

  PLOT("motionControl:JointError:rAnkleRoll:angle", angles[Joints::rAnkleRoll].toDegrees());

  //#ifdef LOGGING
  //  DEBUG_RESPONSE("representation:JointError:CSVLog")
  //  {
  //    LOG("JointError", "headYaw", angles[Joints::headYaw]);
  //    LOG("JointError", "headPitch", angles[Joints::headPitch]);
  //
  //    LOG("JointError", "lShoulderPitch", angles[Joints::lShoulderPitch]);
  //    LOG("JointError", "lShoulderRoll", angles[Joints::lShoulderRoll]);
  //    LOG("JointError", "lElbowYaw", angles[Joints::lElbowYaw]);
  //    LOG("JointError", "lElbowRoll", angles[Joints::lElbowRoll]);
  //    LOG("JointError", "lWristYaw", angles[Joints::lWristYaw]);
  //    LOG("JointError", "lHand", angles[Joints::lHand]);
  //
  //    LOG("JointError", "rShoulderPitch", angles[Joints::rShoulderPitch]);
  //    LOG("JointError", "rShoulderRoll", angles[Joints::rShoulderRoll]);
  //    LOG("JointError", "rElbowYaw", angles[Joints::rElbowYaw]);
  //    LOG("JointError", "rElbowRoll", angles[Joints::rElbowRoll]);
  //    LOG("JointError", "rWristYaw", angles[Joints::rWristYaw]);
  //    LOG("JointError", "rHand", angles[Joints::rHand]);
  //
  //    LOG("JointError", "lHipYawPitch", angles[Joints::lHipYawPitch]);
  //    LOG("JointError", "lHipRoll", angles[Joints::lHipRoll]);
  //    LOG("JointError", "lHipPitch", angles[Joints::lHipPitch]);
  //    LOG("JointError", "lKneePitch", angles[Joints::lKneePitch]);
  //    LOG("JointError", "lAnklePitch", angles[Joints::lAnklePitch]);
  //    LOG("JointError", "lAnkleRoll", angles[Joints::lAnkleRoll]);
  //
  //    LOG("JointError", "rHipYawPitch", angles[Joints::rHipYawPitch]);
  //    LOG("JointError", "rHipRoll", angles[Joints::rHipRoll]);
  //    LOG("JointError", "rHipPitch", angles[Joints::rHipPitch]);
  //    LOG("JointError", "rKneePitch", angles[Joints::rKneePitch]);
  //    LOG("JointError", "rAnklePitch", angles[Joints::rAnklePitch]);
  //    LOG("JointError", "rAnkleRoll", angles[Joints::rAnkleRoll]);
  //  }
  //#endif
}
