#include "JointRequest.h"
#include "Tools/Debugging/DebugDrawings.h"

void JointRequest::draw() const
{
  PLOT("representation:JointRequest:headYaw:angle", angles[Joints::headYaw].toDegrees());
  PLOT("representation:JointRequest:headYaw:stiffness", stiffnessData.stiffnesses[Joints::headYaw]);

  PLOT("representation:JointRequest:headPitch:angle", angles[Joints::headPitch].toDegrees());
  PLOT("representation:JointRequest:headPitch:stiffness", stiffnessData.stiffnesses[Joints::headPitch]);

  PLOT("representation:JointRequest:lShoulderPitch:angle", angles[Joints::lShoulderPitch].toDegrees());
  PLOT("representation:JointRequest:lShoulderPitch:stiffness", stiffnessData.stiffnesses[Joints::lShoulderPitch]);

  PLOT("representation:JointRequest:lShoulderRoll:angle", angles[Joints::lShoulderRoll].toDegrees());
  PLOT("representation:JointRequest:lShoulderRoll:stiffness", stiffnessData.stiffnesses[Joints::lShoulderRoll]);

  PLOT("representation:JointRequest:lElbowYaw:angle", angles[Joints::lElbowYaw].toDegrees());
  PLOT("representation:JointRequest:lElbowYaw:stiffness", stiffnessData.stiffnesses[Joints::lElbowYaw]);

  PLOT("representation:JointRequest:lElbowRoll:angle", angles[Joints::lElbowRoll].toDegrees());
  PLOT("representation:JointRequest:lElbowRoll:stiffness", stiffnessData.stiffnesses[Joints::lElbowRoll]);

  PLOT("representation:JointRequest:lWristYaw:angle", angles[Joints::lWristYaw].toDegrees());
  PLOT("representation:JointRequest:lWristYaw:stiffness", stiffnessData.stiffnesses[Joints::lWristYaw]);

  PLOT("representation:JointRequest:lHand:angle", angles[Joints::lHand].toDegrees());
  PLOT("representation:JointRequest:lHand:stiffness", stiffnessData.stiffnesses[Joints::lHand]);

  PLOT("representation:JointRequest:rShoulderPitch:angle", angles[Joints::rShoulderPitch].toDegrees());
  PLOT("representation:JointRequest:rShoulderPitch:stiffness", stiffnessData.stiffnesses[Joints::rShoulderPitch]);

  PLOT("representation:JointRequest:rShoulderRoll:angle", angles[Joints::rShoulderRoll].toDegrees());
  PLOT("representation:JointRequest:rShoulderRoll:stiffness", stiffnessData.stiffnesses[Joints::rShoulderRoll]);

  PLOT("representation:JointRequest:rElbowYaw:angle", angles[Joints::rElbowYaw].toDegrees());
  PLOT("representation:JointRequest:rElbowYaw:stiffness", stiffnessData.stiffnesses[Joints::rElbowYaw]);

  PLOT("representation:JointRequest:rElbowRoll:angle", angles[Joints::rElbowRoll].toDegrees());
  PLOT("representation:JointRequest:rElbowRoll:stiffness", stiffnessData.stiffnesses[Joints::rElbowRoll]);

  PLOT("representation:JointRequest:rWristYaw:angle", angles[Joints::rWristYaw].toDegrees());
  PLOT("representation:JointRequest:rWristYaw:stiffness", stiffnessData.stiffnesses[Joints::rWristYaw]);

  PLOT("representation:JointRequest:rHand:angle", angles[Joints::rHand].toDegrees());
  PLOT("representation:JointRequest:rHand:stiffness", stiffnessData.stiffnesses[Joints::rHand]);

  PLOT("representation:JointRequest:lHipYawPitch:angle", angles[Joints::lHipYawPitch].toDegrees());
  PLOT("representation:JointRequest:lHipYawPitch:stiffness", stiffnessData.stiffnesses[Joints::lHipYawPitch]);

  PLOT("representation:JointRequest:lHipRoll:angle", angles[Joints::lHipRoll].toDegrees());
  PLOT("representation:JointRequest:lHipRoll:stiffness", stiffnessData.stiffnesses[Joints::lHipRoll]);

  PLOT("representation:JointRequest:lHipPitch:angle", angles[Joints::lHipPitch].toDegrees());
  PLOT("representation:JointRequest:lHipPitch:stiffness", stiffnessData.stiffnesses[Joints::lHipPitch]);

  PLOT("representation:JointRequest:lKneePitch:angle", angles[Joints::lKneePitch].toDegrees());
  PLOT("representation:JointRequest:lKneePitch:stiffness", stiffnessData.stiffnesses[Joints::lKneePitch]);

  PLOT("representation:JointRequest:lAnklePitch:angle", angles[Joints::lAnklePitch].toDegrees());
  PLOT("representation:JointRequest:lAnklePitch:stiffness", stiffnessData.stiffnesses[Joints::lAnklePitch]);

  PLOT("representation:JointRequest:lAnkleRoll:angle", angles[Joints::lAnkleRoll].toDegrees());
  PLOT("representation:JointRequest:lAnkleRoll:stiffness", stiffnessData.stiffnesses[Joints::lAnkleRoll]);

  PLOT("representation:JointRequest:rHipYawPitch:angle", angles[Joints::rHipYawPitch].toDegrees());
  PLOT("representation:JointRequest:rHipYawPitch:stiffness", stiffnessData.stiffnesses[Joints::rHipYawPitch]);

  PLOT("representation:JointRequest:rHipRoll:angle", angles[Joints::rHipRoll].toDegrees());
  PLOT("representation:JointRequest:rHipRoll:stiffness", stiffnessData.stiffnesses[Joints::rHipRoll]);

  PLOT("representation:JointRequest:rHipPitch:angle", angles[Joints::rHipPitch].toDegrees());
  PLOT("representation:JointRequest:rHipPitch:stiffness", stiffnessData.stiffnesses[Joints::rHipPitch]);

  PLOT("representation:JointRequest:rKneePitch:angle", angles[Joints::rKneePitch].toDegrees());
  PLOT("representation:JointRequest:rKneePitch:stiffness", stiffnessData.stiffnesses[Joints::rKneePitch]);

  PLOT("representation:JointRequest:rAnklePitch:angle", angles[Joints::rAnklePitch].toDegrees());
  PLOT("representation:JointRequest:rAnklePitch:stiffness", stiffnessData.stiffnesses[Joints::rAnklePitch]);

  PLOT("representation:JointRequest:rAnkleRoll:angle", angles[Joints::rAnkleRoll].toDegrees());
  PLOT("representation:JointRequest:rAnkleRoll:stiffness", stiffnessData.stiffnesses[Joints::rAnkleRoll]);
}
