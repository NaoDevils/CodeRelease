/**
* @file CoPProvider.cpp
*
* This file implements a module that provides information about the current ZMP.
*
*/

#include "CoPProvider.h"
//#define LOGGING
//#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"


void CoPProvider::update(ZMPModel &zmpModel)
{
#if 0
	Vector3_D<> lCoP_FCS(theFilteredSensorData.data[SensorData::lCoPX], theFilteredSensorData.data[SensorData::lCoPY], 0);
	Vector3_D<> rCoP_FCS(theFilteredSensorData.data[SensorData::rCoPX], theFilteredSensorData.data[SensorData::rCoPY], 0);
#else
  Vector3f lCoP_FCS = Vector3f::Zero();
  Vector3f rCoP_FCS = Vector3f::Zero();

  float leftSum = theFsrSensorData.left[FsrSensorData::fl] +
    theFsrSensorData.left[FsrSensorData::fr] +
    theFsrSensorData.left[FsrSensorData::bl] +
    theFsrSensorData.left[FsrSensorData::br];
  
  if (leftSum > 0)
  {
    lCoP_FCS.x() = (theFsrSensorData.left[FsrSensorData::fl] * 0.07025f +
      theFsrSensorData.left[FsrSensorData::fr] * 0.07025f +
      theFsrSensorData.left[FsrSensorData::bl] * -0.03025f +
      theFsrSensorData.left[FsrSensorData::br] * -0.02965f) / leftSum;
    lCoP_FCS.y() = (theFsrSensorData.left[FsrSensorData::fl] * 0.0299f +
      theFsrSensorData.left[FsrSensorData::fr] * -0.0231f +
      theFsrSensorData.left[FsrSensorData::bl] * 0.0299f +
      theFsrSensorData.left[FsrSensorData::br] * -0.0191f) / leftSum;
  }

  float rightSum = theFsrSensorData.right[FsrSensorData::fl] +
    theFsrSensorData.right[FsrSensorData::fr] +
    theFsrSensorData.right[FsrSensorData::bl] +
    theFsrSensorData.right[FsrSensorData::br];

  if (rightSum > 0)
  {
    rCoP_FCS.x() = (theFsrSensorData.right[FsrSensorData::fl] * 0.07025f +
      theFsrSensorData.right[FsrSensorData::fr] * 0.07025f +
      theFsrSensorData.right[FsrSensorData::bl] * -0.03025f +
      theFsrSensorData.right[FsrSensorData::br] * -0.02965f) / rightSum;
    rCoP_FCS.y() = (theFsrSensorData.right[FsrSensorData::fl] * 0.0231f +
      theFsrSensorData.right[FsrSensorData::fr] * -0.0299f +
      theFsrSensorData.right[FsrSensorData::bl] * 0.0191f +
      theFsrSensorData.right[FsrSensorData::br] * -0.0299f) / rightSum;
  }
#endif
	// convert to robot coordinate system
#if 1
	Pose3f footLeft(theRobotModel.limbs[Limbs::footLeft].rotation, (theRobotModel.limbs[Limbs::footLeft].translation / 1000));
	Pose3f footRight(theRobotModel.limbs[Limbs::footRight].rotation, (theRobotModel.limbs[Limbs::footRight].translation / 1000));
#else
  Pose3D_D footLeft(RotationMatrix_D(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].r,
    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].ry,
    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].rx),
    Vector3_D<double>(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x / 1000,
    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].y / 1000,
    theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].z / 1000));
	Pose3D_D footRight(RotationMatrix_D(theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].r,
    theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].ry,
    theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].rx),
    Vector3_D<double>(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x / 1000,
    theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].y / 1000,
    theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].z / 1000));
#endif

	Vector3f lCoP_RCS = footLeft * lCoP_FCS;
	Vector3f rCoP_RCS = footRight * rCoP_FCS;

	float robotMass = theFsrSensorData.leftTotal + theFsrSensorData.rightTotal;
	
	float lfac=0.5f, rfac=0.5f;

	if (robotMass>0.001)
	{
		lfac = theFsrSensorData.leftTotal / robotMass;
		rfac = theFsrSensorData.rightTotal / robotMass;
	}
	if (theWalkingInfo.kickPhase!=freeLegNA)
	{
		// Weights delivered by Aldebaran are wrong when standing on one leg
		if (theWalkingInfo.lastUsedFootPositions.onFloor[LEFT_FOOT])
		{
			lfac = 1.0f;
			rfac = 0.0f;
		}
		else 
		{
			lfac = 0.0f;
			rfac = 1.0f;
		}
	}

	zmpModel.zmp_acc = lCoP_RCS * lfac + rCoP_RCS * rfac;

	Pose3f footLeft_WCS(theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x, theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].y, theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].z);
	footLeft_WCS.rotation=RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].r);
	Pose3f footRight_WCS(theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].x, theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].y, theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].z);
	footRight_WCS.rotation=RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].r);

	Vector3f lCoP_WCS = footLeft_WCS * lCoP_FCS;
	Vector3f rCoP_WCS = footRight_WCS * rCoP_FCS;

	zmpModel.ZMP_WCS = lCoP_WCS * lfac + rCoP_WCS * rfac;
	
	PLOT("module:CoPProvider:CoPx", zmpModel.zmp_acc.x());
	PLOT("module:CoPProvider:CoPy", zmpModel.zmp_acc.y());

	PLOT("module:CoPProvider:CoPxWCS", zmpModel.ZMP_WCS.x());
	PLOT("module:CoPProvider:CoPyWCS", zmpModel.ZMP_WCS.y());

	PLOT("module:CoPProvider:lCoP_FCS x", lCoP_FCS.x());
	PLOT("module:CoPProvider:lCoP_FCS y", lCoP_FCS.y());

	PLOT("module:CoPProvider:rCoP_FCS x", rCoP_FCS.x());
	PLOT("module:CoPProvider:rCoP_FCS y", rCoP_FCS.y());

	PLOT("module:CoPProvider:lWeight", theFsrSensorData.leftTotal);
	PLOT("module:CoPProvider:rWeight", theFsrSensorData.rightTotal);

}

MAKE_MODULE(CoPProvider, sensing)

