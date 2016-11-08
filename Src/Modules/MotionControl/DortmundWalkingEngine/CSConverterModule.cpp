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
		theFreeLegPhaseParams,
		theTorsoMatrix,
    theRobotModel,
    theRobotInfo,
    theFootSteps)
{
}

void CSConverterModule::update(KinematicRequest &kinematicRequest)
{
	converter.updateKinematicRequest(kinematicRequest);
  PLOT("module:CSConverter:TargetCoM.x", theTargetCoM.x);
  PLOT("module:CSConverter:TargetCoM.y", theTargetCoM.y);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].x", theFootpositions.footPos[LEFT_FOOT].x);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].y", theFootpositions.footPos[LEFT_FOOT].y);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].z", theFootpositions.footPos[LEFT_FOOT].z);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].r", theFootpositions.footPos[LEFT_FOOT].r);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].rx", theFootpositions.footPos[LEFT_FOOT].rx);
  PLOT("module:CSConverter:Footpos[LEFT_FOOT].ry", theFootpositions.footPos[LEFT_FOOT].ry);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].x", theFootpositions.footPos[RIGHT_FOOT].x);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].y", theFootpositions.footPos[RIGHT_FOOT].y);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].z", theFootpositions.footPos[RIGHT_FOOT].z);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].r", theFootpositions.footPos[RIGHT_FOOT].r);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].rx", theFootpositions.footPos[RIGHT_FOOT].rx);
  PLOT("module:CSConverter:Footpos[RIGHT_FOOT].ry", theFootpositions.footPos[RIGHT_FOOT].ry);
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