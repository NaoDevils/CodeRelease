#include "FLIPMObservedStateProvider.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/DebugDrawings.h"


void FLIPMObservedStateProvider::update(FLIPMObservedState &theFLIPMObservedState)
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

  Point actualCoMRCS(theRobotModel.centerOfMass.x() / 1000, theRobotModel.centerOfMass.y() / 1000, (theRobotModel.centerOfMass.z()) / 1000, 0);

  (Point &)theFLIPMObservedState.actualCoM_MRE = theWalkingInfo.toWorldCoords(actualCoMRCS);

  actualCoMRCS.rotateAroundX(theIMUModel.orientation.x());
  actualCoMRCS.rotateAroundY(theIMUModel.orientation.y());

  (Point &)theFLIPMObservedState.actualCoM_IMU = theWalkingInfo.toWorldCoords(actualCoMRCS);

  theFLIPMObservedState.actualAcc.x() = theIMUModel.acceleration.x();
  theFLIPMObservedState.actualAcc.y() = theIMUModel.acceleration.y();
  theFLIPMObservedState.actualAcc.z() = theIMUModel.acceleration.z();

  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.x", theFLIPMObservedState.actualCoM_IMU.x);
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.y", theFLIPMObservedState.actualCoM_IMU.y);
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_IMU.z", theFLIPMObservedState.actualCoM_IMU.z);
                                                                                                      
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.x", theFLIPMObservedState.actualCoM_MRE.x);
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.y", theFLIPMObservedState.actualCoM_MRE.y);
  PLOT("module:FLIPMObservedStateProvider:ActualCoMFLIPM_MRE.z", theFLIPMObservedState.actualCoM_MRE.z);

  PLOT("module:FLIPMObservedStateProvider:ActualAcc.x", theFLIPMObservedState.actualAcc.x());
  PLOT("module:FLIPMObservedStateProvider:ActualAcc.y", theFLIPMObservedState.actualAcc.y());
  PLOT("module:FLIPMObservedStateProvider:ActualAcc.z", theFLIPMObservedState.actualAcc.z());
}
MAKE_MODULE(FLIPMObservedStateProvider, dortmundWalkingEngine)