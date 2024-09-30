/**
* @file CoPProvider.cpp
*
* This file implements a module that provides information about the current ZMP.
*
* @author <A href="arne.arne@tu-dortmund.de">Arne Moos</A>
* 
*/

#include "CoPProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

//#define LOGGING
//#include "Tools/Debugging/CSVLogger.h"

using namespace DWE;

void CoPProvider::update(ZMPModel& zmpModel)
{
  const FsrSensorData& fsrData = useFsrModel ? (FsrSensorData&)theFsrModelData : theFsrSensorData;

  float robotMass = fsrData.leftTotal + fsrData.rightTotal;
  float lfac = 0.5f, rfac = 0.5f;
  if (robotMass > 0.001)
  {
    lfac = fsrData.leftTotal / robotMass;
    rfac = fsrData.rightTotal / robotMass;
  }

  Vector3f lCoP_FCS = Vector3f::Zero();
  Vector3f rCoP_FCS = Vector3f::Zero();

  float leftSum = fsrData.left[FsrSensorData::fl] + fsrData.left[FsrSensorData::fr] + fsrData.left[FsrSensorData::bl] + fsrData.left[FsrSensorData::br];

  if (leftSum > 0)
  {
    // clang-format off
    lCoP_FCS.x() = (fsrData.left[FsrSensorData::fl] * 0.07025f +
                    fsrData.left[FsrSensorData::fr] * 0.07025f +
                    fsrData.left[FsrSensorData::bl] * -0.03025f +
                    fsrData.left[FsrSensorData::br] * -0.02965f) / leftSum;
    lCoP_FCS.y() = (fsrData.left[FsrSensorData::fl] * 0.0299f +
                    fsrData.left[FsrSensorData::fr] * -0.0231f +
                    fsrData.left[FsrSensorData::bl] * 0.0299f +
                    fsrData.left[FsrSensorData::br] * -0.0191f) / leftSum;
    // clang-format on
  }

  float rightSum = fsrData.right[FsrSensorData::fl] + fsrData.right[FsrSensorData::fr] + fsrData.right[FsrSensorData::bl] + fsrData.right[FsrSensorData::br];

  if (rightSum > 0)
  {
    // clang-format off
    rCoP_FCS.x() = (fsrData.right[FsrSensorData::fr] * 0.07025f +
                    fsrData.right[FsrSensorData::fl] * 0.07025f +
                    fsrData.right[FsrSensorData::br] * -0.03025f +
                    fsrData.right[FsrSensorData::bl] * -0.02965f) / rightSum;
    rCoP_FCS.y() = (fsrData.right[FsrSensorData::fr] * -0.0299f +
                    fsrData.right[FsrSensorData::fl] * 0.0231f +
                    fsrData.right[FsrSensorData::br] * -0.0299f +
                    fsrData.right[FsrSensorData::bl] * 0.0191f) / rightSum;
    // clang-format on
  }
  Vector3f CoP_FCS = lCoP_FCS * lfac - lCoP_FCS * rfac;
  zmpModel.ZMP_FCS = CoP_FCS.head<2>();

  // convert to robot coordinate system
  zmpModel.feetPose.left = Pose3f(theRobotModel.limbs[Limbs::footLeft].rotation, (theRobotModel.limbs[Limbs::footLeft].translation / 1000));
  zmpModel.feetPose.right = Pose3f(theRobotModel.limbs[Limbs::footRight].rotation, (theRobotModel.limbs[Limbs::footRight].translation / 1000));

  Vector3f lCoP_RCS = zmpModel.feetPose.left * lCoP_FCS;
  Vector3f rCoP_RCS = zmpModel.feetPose.right * rCoP_FCS;

  zmpModel.ZMP_RCS = lCoP_RCS * lfac + rCoP_RCS * rfac;
  // clang-format off
  Pose3f footLeft_WCS(
      theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].x,
      theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].y,
      theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].z);
  Pose3f footRight_WCS(
      theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].x,
      theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].y,
      theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].z);
  // clang-format on
  footLeft_WCS.rotation = RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[LEFT_FOOT].r);
  footRight_WCS.rotation = RotationMatrix(0.f, 0.f, theWalkingInfo.lastUsedFootPositions.footPos[RIGHT_FOOT].r);

  Vector3f lCoP_WCS = footLeft_WCS * lCoP_FCS;
  Vector3f rCoP_WCS = footRight_WCS * rCoP_FCS;
  zmpModel.ZMP_WCS = lCoP_WCS * lfac + rCoP_WCS * rfac;

  PLOT("module:CoPProvider:CoPx", zmpModel.ZMP_RCS.x());
  PLOT("module:CoPProvider:CoPy", zmpModel.ZMP_RCS.y());

  PLOT("module:CoPProvider:CoPxWCS", zmpModel.ZMP_WCS.x());
  PLOT("module:CoPProvider:CoPyWCS", zmpModel.ZMP_WCS.y());

  PLOT("module:CoPProvider:lCoP_FCS x", lCoP_FCS.x());
  PLOT("module:CoPProvider:lCoP_FCS y", lCoP_FCS.y());

  PLOT("module:CoPProvider:rCoP_FCS x", rCoP_FCS.x());
  PLOT("module:CoPProvider:rCoP_FCS y", rCoP_FCS.y());
}

MAKE_MODULE(CoPProvider, sensing)
