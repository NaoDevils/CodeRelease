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
  PLOT("representation:JointError:headYaw:angle", angles[Joints::headYaw].toDegrees());

  PLOT("representation:JointError:headPitch:angle", angles[Joints::headPitch].toDegrees());

  PLOT("representation:JointError:lShoulderPitch:angle", angles[Joints::lShoulderPitch].toDegrees());

  PLOT("representation:JointError:lShoulderRoll:angle", angles[Joints::lShoulderRoll].toDegrees());

  PLOT("representation:JointError:lElbowYaw:angle", angles[Joints::lElbowYaw].toDegrees());

  PLOT("representation:JointError:lElbowRoll:angle", angles[Joints::lElbowRoll].toDegrees());

  PLOT("representation:JointError:lWristYaw:angle", angles[Joints::lWristYaw].toDegrees());

  PLOT("representation:JointError:lHand:angle", angles[Joints::lHand].toDegrees());

  PLOT("representation:JointError:rShoulderPitch:angle", angles[Joints::rShoulderPitch].toDegrees());

  PLOT("representation:JointError:rShoulderRoll:angle", angles[Joints::rShoulderRoll].toDegrees());

  PLOT("representation:JointError:rElbowYaw:angle", angles[Joints::rElbowYaw].toDegrees());

  PLOT("representation:JointError:rElbowRoll:angle", angles[Joints::rElbowRoll].toDegrees());

  PLOT("representation:JointError:rWristYaw:angle", angles[Joints::rWristYaw].toDegrees());

  PLOT("representation:JointError:rHand:angle", angles[Joints::rHand].toDegrees());

  PLOT("representation:JointError:lHipYawPitch:angle", angles[Joints::lHipYawPitch].toDegrees());

  PLOT("representation:JointError:lHipRoll:angle", angles[Joints::lHipRoll].toDegrees());

  PLOT("representation:JointError:lHipPitch:angle", angles[Joints::lHipPitch].toDegrees());

  PLOT("representation:JointError:lKneePitch:angle", angles[Joints::lKneePitch].toDegrees());

  PLOT("representation:JointError:lAnklePitch:angle", angles[Joints::lAnklePitch].toDegrees());

  PLOT("representation:JointError:lAnkleRoll:angle", angles[Joints::lAnkleRoll].toDegrees());

  PLOT("representation:JointError:rHipYawPitch:angle", angles[Joints::rHipYawPitch].toDegrees());

  PLOT("representation:JointError:rHipRoll:angle", angles[Joints::rHipRoll].toDegrees());

  PLOT("representation:JointError:rHipPitch:angle", angles[Joints::rHipPitch].toDegrees());

  PLOT("representation:JointError:rKneePitch:angle", angles[Joints::rKneePitch].toDegrees());

  PLOT("representation:JointError:rAnklePitch:angle", angles[Joints::rAnklePitch].toDegrees());

  PLOT("representation:JointError:rAnkleRoll:angle", angles[Joints::rAnkleRoll].toDegrees());

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
