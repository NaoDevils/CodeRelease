#include "IMUCoMProvider.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/DebugDrawings.h"

void IMUCoMProvider::update(ActualCoM& theActualCoM)
{
  DECLARE_PLOT("module:CoMProvider:ActualCoM.x");
  DECLARE_PLOT("module:CoMProvider:ActualCoM.y");

  static bool running = false;
  if (!theFootSteps.running && running)
  {
    running = false;
    (Point&)theActualCoM = Point(0.0, 0.0);
    return;
  }
  else if (theFootSteps.running && !running)
  {
    running = true;
  }


  Point rcs = theWalkingInfo.toRobotCoords(theTargetCoM);

  rcs.rotateAroundX(theJoinedIMUData.imuData[anglesource].angle.x());
  rcs.rotateAroundY(theJoinedIMUData.imuData[anglesource].angle.y());

  (Point&)theActualCoM = theWalkingInfo.toWorldCoords(rcs);

  PLOT("module:CoMProvider:ActualCoM.x", theActualCoM.x);
  PLOT("module:CoMProvider:ActualCoM.y", theActualCoM.y);
}

MAKE_MODULE(IMUCoMProvider, dortmundWalkingEngine)
