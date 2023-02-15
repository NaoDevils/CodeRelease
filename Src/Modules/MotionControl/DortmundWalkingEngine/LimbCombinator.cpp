#include "LimbCombinator.h"
#include <cmath>

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace DWE;

LimbCombinator::LimbCombinator()
{
  init = false;
  lastInKick = false;
  for (int i = 0; i < Joints::numOfJoints; i++)
    for (int j = 0; j < 200; j++)
      angleoffset[i][j] = 0;
  lastOutFilterOrder = theWalkingEngineParams.outFilterOrder;
  //	waitLegHardness=0;
  //	waitLegSoftness=theWalkingEngineParams.softnessDelay;
}


LimbCombinator::~LimbCombinator() {}

int LimbCombinator::walkingEngineTime = 0;

void LimbCombinator::update(WalkingEngineOutput& walkingEngineOutput)
{
  walkingEngineTime++;

  float delayMeasurementOffset = 0;

  MODIFY("delayMeasurementOffset", delayMeasurementOffset);

  if (!init || theWalkingEngineParams.outFilterOrder != lastOutFilterOrder)
  {
    for (int i = 0; i < Joints::numOfJoints; i++)
    {
      filter[i].createBuffer(theWalkingEngineParams.outFilterOrder);
    }

    lHipPitchTargetAngle.clear();
    rHipPitchTargetAngle.clear();
    for (size_t i = 0; i < lHipPitchTargetAngle.capacity(); i++)
    {
      lHipPitchTargetAngle.push_front(theJointSensorData.angles[Joints::lHipPitch]);
      rHipPitchTargetAngle.push_front(theJointSensorData.angles[Joints::rHipPitch]);
    }
    init = true;
  }

  for (int i = 0; i < Joints::numOfJoints; i++)
  {
    walkingEngineOutput.angles[i] = filter[i].nextValue(theKinematicOutput.angles[i]);
    if (std::isnan((float)walkingEngineOutput.angles[i]))
    {
      OutMapFile map("logs/limbCombinator.cfg");
      map << "fpL.x " << theFootpositions.footPos[0].x << "\n";
      map << "fpL.y " << theFootpositions.footPos[0].y << "\n";
      map << "fpL.z " << theFootpositions.footPos[0].z << "\n";
      map << "fpR.x " << theFootpositions.footPos[1].x << "\n";
      map << "fpR.y " << theFootpositions.footPos[1].y << "\n";
      map << "fpR.y " << theFootpositions.footPos[1].z << "\n";
      map << "tC.x " << theTargetCoM.x << "\n";
      map << "tC.y " << theTargetCoM.y << "\n";
      map << "tC.z " << theTargetCoM.z << "\n";
      map << "sR.x " << theSpeedRequest.translation.x() << "\n";
      map << "sR.y " << theSpeedRequest.translation.y() << "\n";
      map << "sR.r " << theSpeedRequest.rotation << "\n";
      ASSERT(false);
    }
  }

  if (theFootpositions.inKick && !lastInKick)
    timeStampKickStarted = theFrameInfo.time;
  lastInKick = theFootpositions.inKick;

  int timeSinceKickStarted = theFrameInfo.getTimeSince(timeStampKickStarted);

  bool doKickHackHip = useKickHack && timeSinceKickStarted > theFootpositions.timeUntilKickHackHip
      && timeSinceKickStarted < theFootpositions.timeUntilKickHackHip + theFootpositions.kickHackDurationHip;
  bool doKickHackKnee = useKickHack && timeSinceKickStarted > theFootpositions.timeUntilKickHackKnee
      && timeSinceKickStarted < theFootpositions.timeUntilKickHackKnee + theFootpositions.kickHackDurationKnee;

  if (doKickHackHip)
    applyKickHackHip(walkingEngineOutput);
  if (doKickHackKnee)
    applyKickHackKnee(walkingEngineOutput);

  if (theArmMovement.usearms)
  {
    for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
      walkingEngineOutput.angles[i] = theArmMovement.angles[i];
  }
  else // if theArmMovement.usearms == false
  {
    for (int i = Joints::firstArmJoint; i < Joints::lHipYawPitch; i++)
      walkingEngineOutput.angles[i] = JointAngles::ignore;
  }

  walkingEngineOutput.odometryOffset = theWalkingInfo.odometryOffset;
  walkingEngineOutput.isLeavingPossible = theWalkingInfo.isLeavingPossible;
  walkingEngineOutput.speed.translation.x() = theSpeedInfo.speed.translation.x() * 1000;
  walkingEngineOutput.speed.translation.y() = theSpeedInfo.speed.translation.y() * 1000;
  walkingEngineOutput.speed.rotation = theSpeedInfo.speed.rotation;
  walkingEngineOutput.offsetToRobotPoseAfterPreview = theWalkingInfo.offsetToRobotPoseAfterPreview;

  //set leg hardness
  static unsigned int waitCounter = 0;
  {
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipYawPitch] = theWalkingEngineParams.jointCalibration.legJointHardness[0];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipRoll] = theWalkingEngineParams.jointCalibration.legJointHardness[1];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipPitch] = theWalkingEngineParams.jointCalibration.legJointHardness[2];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lKneePitch] = theWalkingEngineParams.jointCalibration.legJointHardness[3];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lAnklePitch] = theWalkingEngineParams.jointCalibration.legJointHardness[4];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::lAnkleRoll] = theWalkingEngineParams.jointCalibration.legJointHardness[5];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipYawPitch] = theWalkingEngineParams.jointCalibration.legJointHardness[0];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipRoll] = theWalkingEngineParams.jointCalibration.legJointHardness[1];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipPitch] = theWalkingEngineParams.jointCalibration.legJointHardness[2];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rKneePitch] = theWalkingEngineParams.jointCalibration.legJointHardness[3];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rAnklePitch] = theWalkingEngineParams.jointCalibration.legJointHardness[4];
    walkingEngineOutput.stiffnessData.stiffnesses[Joints::rAnkleRoll] = theWalkingEngineParams.jointCalibration.legJointHardness[5];
    waitCounter--;
  }

  // set arm hardness
  for (int i = Joints::lShoulderPitch; i <= Joints::rElbowRoll; i++)
  {
    walkingEngineOutput.stiffnessData.stiffnesses[i] = StiffnessData::useDefault;
  }

  for (int i = 0; i < 12; i++)
  {
    walkingEngineOutput.angles[Joints::lHipYawPitch + i] += theWalkingEngineParams.jointCalibration.jointCalibration[i];
  }

  // compensate joint error, i.e. when a joint is not able to reach its target
  unsigned sensorDelay = std::min<unsigned>(theWalkingEngineParams.jointSensorDelayFrames - 1, MAX_DELAY_FRAMES - 1);
  Angle lHipPitchError = std::max(-5_deg, std::min<Angle>(lHipPitchTargetAngle[sensorDelay] - theJointSensorData.angles[Joints::lHipPitch], 5_deg));
  Angle rHipPitchError = std::max(-5_deg, std::min<Angle>(rHipPitchTargetAngle[sensorDelay] - theJointSensorData.angles[Joints::rHipPitch], 5_deg));
  walkingEngineOutput.angles[Joints::lKneePitch] -= lHipPitchError * hipPitchKneeCompensation;
  walkingEngineOutput.angles[Joints::rKneePitch] -= rHipPitchError * hipPitchKneeCompensation;

  // TODO: replace this with other logging mechanisms
  // TODO: Care about your own problems
  LOG("GlobalAngles", "Joints LHipYawPitch", theJointSensorData.angles[Joints::lHipYawPitch]);
  LOG("GlobalAngles", "Joints LHipRoll", theJointSensorData.angles[Joints::lHipRoll]);
  LOG("GlobalAngles", "Joints LHipPitch", theJointSensorData.angles[Joints::lHipPitch]);
  LOG("GlobalAngles", "Joints LKneePitch", theJointSensorData.angles[Joints::lKneePitch]);
  LOG("GlobalAngles", "Joints LAnklePitch", theJointSensorData.angles[Joints::lAnklePitch]);
  LOG("GlobalAngles", "Joints LAnkleRoll", theJointSensorData.angles[Joints::lAnkleRoll]);

  LOG("GlobalAngles", "Joints RHipYawPitch", theJointSensorData.angles[Joints::rHipYawPitch]);
  LOG("GlobalAngles", "Joints RHipRoll", theJointSensorData.angles[Joints::rHipRoll]);
  LOG("GlobalAngles", "Joints RHipPitch", theJointSensorData.angles[Joints::rHipPitch]);
  LOG("GlobalAngles", "Joints RKneePitch", theJointSensorData.angles[Joints::rKneePitch]);
  LOG("GlobalAngles", "Joints RAnklePitch", theJointSensorData.angles[Joints::rAnklePitch]);
  LOG("GlobalAngles", "Joints RAnkleRoll", theJointSensorData.angles[Joints::rAnkleRoll]);

  LOG("GlobalAngles", "LimbCombinator LHipYawPitch", walkingEngineOutput.angles[Joints::lHipYawPitch]);
  LOG("GlobalAngles", "LimbCombinator LHipRoll", walkingEngineOutput.angles[Joints::lHipRoll]);
  LOG("GlobalAngles", "LimbCombinator LHipPitch", walkingEngineOutput.angles[Joints::lHipPitch]);
  LOG("GlobalAngles", "LimbCombinator LKneePitch", walkingEngineOutput.angles[Joints::lKneePitch]);
  LOG("GlobalAngles", "LimbCombinator LAnklePitch", walkingEngineOutput.angles[Joints::lAnklePitch]);
  LOG("GlobalAngles", "LimbCombinator LAnkleRoll", walkingEngineOutput.angles[Joints::lAnkleRoll]);

  LOG("GlobalAngles", "LimbCombinator RHipYawPitch", walkingEngineOutput.angles[Joints::rHipYawPitch]);
  LOG("GlobalAngles", "LimbCombinator RHipRoll", walkingEngineOutput.angles[Joints::rHipRoll]);
  LOG("GlobalAngles", "LimbCombinator RHipPitch", walkingEngineOutput.angles[Joints::rHipPitch]);
  LOG("GlobalAngles", "LimbCombinator RKneePitch", walkingEngineOutput.angles[Joints::rKneePitch]);
  LOG("GlobalAngles", "LimbCombinator RAnklePitch", walkingEngineOutput.angles[Joints::rAnklePitch]);
  LOG("GlobalAngles", "LimbCombinator RAnkleRoll", walkingEngineOutput.angles[Joints::rAnkleRoll]);

  LOG("GlobalAngles", "Walking Engine Time", walkingEngineTime);

  MARK("GlobalAngles", "Ego-CoM x");
  MARK("GlobalAngles", "Ego-CoM y");
  MARK("GlobalAngles", "Ego-CoM z");
  PLOT("module:LimbCombinator:Offset", angleoffset[Joints::rAnklePitch][12]);
  PLOT("module:LimbCombinator:Target", walkingEngineOutput.angles[Joints::rAnklePitch]);
  PLOT("module:LimbCombinator:Measured", theJointSensorData.angles[Joints::rAnklePitch]);

  lastOutFilterOrder = theWalkingEngineParams.outFilterOrder;
}


void LimbCombinator::applyKickHackHip(WalkingEngineOutput& walkingEngineOutput)
{
  if (theFootpositions.onFloor[LEFT_FOOT])
  {
    walkingEngineOutput.angles[Joints::rHipPitch] = theFootpositions.kickHackHipAngle;
    walkingEngineOutput.angles[Joints::rAnklePitch] = -(walkingEngineOutput.angles[Joints::rKneePitch] + walkingEngineOutput.angles[Joints::rHipPitch]);
  }
  else
  {
    walkingEngineOutput.angles[Joints::lHipPitch] = theFootpositions.kickHackHipAngle;
    walkingEngineOutput.angles[Joints::lAnklePitch] = -(walkingEngineOutput.angles[Joints::lKneePitch] + walkingEngineOutput.angles[Joints::lHipPitch]);
  }
}

void LimbCombinator::applyKickHackKnee(WalkingEngineOutput& walkingEngineOutput)
{
  if (theFootpositions.onFloor[LEFT_FOOT])
  {
    walkingEngineOutput.angles[Joints::rKneePitch] = theFootpositions.kickHackKneeAngle;
    walkingEngineOutput.angles[Joints::rAnklePitch] = -(walkingEngineOutput.angles[Joints::rKneePitch] + walkingEngineOutput.angles[Joints::rHipPitch]);
  }
  else
  {
    walkingEngineOutput.angles[Joints::lKneePitch] = theFootpositions.kickHackKneeAngle;
    walkingEngineOutput.angles[Joints::lAnklePitch] = -(walkingEngineOutput.angles[Joints::lKneePitch] + walkingEngineOutput.angles[Joints::lHipPitch]);
  }
}

MAKE_MODULE(LimbCombinator, dortmundWalkingEngine)
