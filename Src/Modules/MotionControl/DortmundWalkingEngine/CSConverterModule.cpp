/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CSConverterModule.cpp
* Module wrapper for the CSConverter
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "CSConverterModule.h"
#include "Tools/Debugging/DebugDrawings.h"
//#define LOGGING
//#include "Tools/Debugging/CSVLogger.h"


CSConverterModule::CSConverterModule():
converter(
		theFootpositions,
		theTargetCoM,
		theWalkingEngineParams,
		theControllerParams,
		theActualCoMRCS,
		theFallDownState,
		theInertialSensorData,
    theFsrSensorData,
		theBodyTilt,
		theTorsoMatrix,
    theRobotModel,
    theRobotInfo,
    theFootSteps,
    theArmContact,
    theRobotDimensions,
    theWalkCalibration)
{
}

void CSConverterModule::update(KinematicRequest &kinematicRequest)
{
	converter.updateKinematicRequest(kinematicRequest);
  PLOT("module:CSConverter:TargetCoM.x", theTargetCoM.x);
  PLOT("module:CSConverter:TargetCoM.y", theTargetCoM.y);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].x", kinematicRequest.leftFoot[0]);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].y", kinematicRequest.leftFoot[1]);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].z", kinematicRequest.leftFoot[2]);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].r", kinematicRequest.leftFoot[3]);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].rx", kinematicRequest.leftFoot[4]);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].ry", kinematicRequest.leftFoot[5]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].x", kinematicRequest.rightFoot[0]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].y", kinematicRequest.rightFoot[1]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].z", kinematicRequest.rightFoot[2]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].r", kinematicRequest.rightFoot[3]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].rx", kinematicRequest.rightFoot[4]);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].ry", kinematicRequest.rightFoot[5]);
  static float sum = 0;
  sum += theInertialSensorData.gyro.y() * 0.01f;
  PLOT("module:CSConverter:sum", sum);
  /*LOG("Oricomp", "Sum", sum);
  LOG("Oricomp", "angleY", theSensorData.data[SensorData::angleY]);*/



};

void CSConverterModule::update(WalkingInfo &walkingInfo)
{
	converter.updateWalkingInfo(walkingInfo);
};

void CSConverterModule::update(FixedOdometryRobotPose &fixedOdometryRobotPose)
{
	fixedOdometryRobotPose = converter.fixedOdometryRobotPose;
}


MAKE_MODULE(CSConverterModule, dortmundWalkingEngine)