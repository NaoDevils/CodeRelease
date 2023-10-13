#include "FLIPMObservedStateProvider.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/DebugDrawings.h"


void FLIPMObservedStateProvider::update(FLIPMObservedState& theFLIPMObservedState)
{
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.x");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.y");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.z");

  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.x");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.y");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.z");

  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualAcc.x");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualAcc.y");
  DECLARE_PLOT("module:FLIPMObservedStateProvider:ActualAcc.z");

  Point coM_MRE_RCS(theRobotModel.centerOfMass.x() / 1000, theRobotModel.centerOfMass.y() / 1000, (theRobotModel.centerOfMass.z()) / 1000, 0);

  Point CoM_MRE_WCS = theWalkingInfo.toWorldCoords(coM_MRE_RCS);
  theFLIPMObservedState.actualCoM_MRE.x() = CoM_MRE_WCS.x;
  theFLIPMObservedState.actualCoM_MRE.y() = CoM_MRE_WCS.y;
  theFLIPMObservedState.actualCoM_MRE.z() = CoM_MRE_WCS.z;

  Point coM_IMU_RCS(coM_MRE_RCS);
  coM_IMU_RCS.rotateAroundX(theJoinedIMUData.imuData[anglesource].angle.x());
  coM_IMU_RCS.rotateAroundY(theJoinedIMUData.imuData[anglesource].angle.y());
  Point CoM_IMU_WCS = theWalkingInfo.toWorldCoords(coM_IMU_RCS);
  theFLIPMObservedState.actualCoM_IMU.x() = CoM_IMU_WCS.x;
  theFLIPMObservedState.actualCoM_IMU.y() = CoM_IMU_WCS.y;
  theFLIPMObservedState.actualCoM_IMU.z() = CoM_IMU_WCS.z;

  theFLIPMObservedState.actualAcc.x() = theJoinedIMUData.imuData[anglesource].acc.x();
  theFLIPMObservedState.actualAcc.y() = theJoinedIMUData.imuData[anglesource].acc.y();
  theFLIPMObservedState.actualAcc.z() = theJoinedIMUData.imuData[anglesource].acc.z();

  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.x", theFLIPMObservedState.actualCoM_IMU.x());
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.y", theFLIPMObservedState.actualCoM_IMU.y());
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.z", theFLIPMObservedState.actualCoM_IMU.z());

  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.x", theFLIPMObservedState.actualCoM_MRE.x());
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.y", theFLIPMObservedState.actualCoM_MRE.y());
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.z", theFLIPMObservedState.actualCoM_MRE.z());

  PLOT("module:FLIPMObservedStateProvider:ActualAcc.x", theFLIPMObservedState.actualAcc.x());
  PLOT("module:FLIPMObservedStateProvider:ActualAcc.y", theFLIPMObservedState.actualAcc.y());
  PLOT("module:FLIPMObservedStateProvider:ActualAcc.z", theFLIPMObservedState.actualAcc.z());
}

MAKE_MODULE(FLIPMObservedStateProvider, dortmundWalkingEngine)
