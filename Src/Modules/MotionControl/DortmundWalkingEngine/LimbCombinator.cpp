#include "LimbCombinator.h"


#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Debugging/DebugDrawings.h"



LimbCombinator::LimbCombinator()
{
	init=false;
	for (int i=0; i<Joints::numOfJoints; i++)
		for (int j=0; j<200; j++)
			angleoffset[i][j]=0;
//	waitLegHardness=0;
//	waitLegSoftness=theWalkingEngineParams.softnessDelay;
}

 
LimbCombinator::~LimbCombinator()
{
}

int LimbCombinator::walkingEngineTime=0;

void LimbCombinator::update(WalkingEngineOutput& walkingEngineOutput)
{
	walkingEngineTime++;

	float delayMeasurementOffset=0;

	MODIFY("delayMeasurementOffset", delayMeasurementOffset);

	if (!init)
	{
		for (int i = 0; i < Joints::numOfJoints; i++)
		{
			filter[i].createBuffer(theWalkingEngineParams.outFilterOrder);
		}
		init=true;
	}

	for (int i=0; i<Joints::numOfJoints; i++)
    {
	  walkingEngineOutput.angles[i]=filter[i].nextValue(theKinematicOutput.angles[i]);
	  if (walkingEngineOutput.angles[i] != walkingEngineOutput.angles[i])
	  {
	    ASSERT(walkingEngineOutput.angles[i] == walkingEngineOutput.angles[i]);
	  }
    }

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


	
	walkingEngineOutput.odometryOffset=theWalkingInfo.odometryOffset;
	walkingEngineOutput.isLeavingPossible=theWalkingInfo.isLeavingPossible;
	walkingEngineOutput.speed.translation.x()=theSpeedInfo.speed.translation.x()*1000;
	walkingEngineOutput.speed.translation.y()=theSpeedInfo.speed.translation.y()*1000;
	walkingEngineOutput.speed.rotation=theSpeedInfo.speed.rotation;
  
	walkingEngineOutput.offsetToRobotPoseAfterPreview = theWalkingInfo.offsetToRobotPoseAfterPreview;
//set leg hardness
	static unsigned int waitCounter=0;
	{
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipYawPitch]=theWalkingEngineParams.jointCalibration.legJointHardness[0];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipRoll]=theWalkingEngineParams.jointCalibration.legJointHardness[1];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lHipPitch]=theWalkingEngineParams.jointCalibration.legJointHardness[2];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lKneePitch]=theWalkingEngineParams.jointCalibration.legJointHardness[3];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lAnklePitch]=theWalkingEngineParams.jointCalibration.legJointHardness[4];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::lAnkleRoll]=theWalkingEngineParams.jointCalibration.legJointHardness[5];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipYawPitch]=theWalkingEngineParams.jointCalibration.legJointHardness[0];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipRoll]=theWalkingEngineParams.jointCalibration.legJointHardness[1];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rHipPitch]=theWalkingEngineParams.jointCalibration.legJointHardness[2];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rKneePitch]=theWalkingEngineParams.jointCalibration.legJointHardness[3];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rAnklePitch]=theWalkingEngineParams.jointCalibration.legJointHardness[4];
		walkingEngineOutput.stiffnessData.stiffnesses[Joints::rAnkleRoll]=theWalkingEngineParams.jointCalibration.legJointHardness[5];
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
}

MAKE_MODULE(LimbCombinator, dortmundWalkingEngine)
