#include "CoMProvider.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Sensing/RobotModel.h"

using namespace std;

CoMProvider::CoMProvider(		
                         const JointAngles				&theJointAngles,
                         const WalkingEngineParams	&theWalkingEngineParams,
                         const JointRequest			&theJointRequest,
                         const FootSteps				&theFootSteps,
                         const RobotModel       &theRobotModel,
                         const ActualCoMRCS			&theActualCoMRCS):
  //theJointAngles(theJointAngles), all unused
//theWalkingEngineParams(theWalkingEngineParams),
//theJointRequest(theJointRequest),
theFootSteps(theFootSteps),
theRobotModel(theRobotModel),
theActualCoMRCS(theActualCoMRCS)
{
}


CoMProvider::~CoMProvider(void)
{

}


void CoMProvider::updateActualCoM(ActualCoM &theActualCoM)
{
	for (int i=0; i<theFootSteps.getNumOfSteps(); i++)
    footPositions.push_back(theFootSteps.getStep(i));
  if (footPositions.empty()) return;

  static bool running = false;
  if (!theFootSteps.running && running)
  {
    footPositions.clear();
    running = false;
    return;
  }
  else if (theFootSteps.running && !running)
  {
    running = true;
  }

  Pose3f measuredCoM(Vector3f(theActualCoMRCS.x, theActualCoMRCS.y, theActualCoMRCS.z));
  Pose3f step;
  Pose3f footPos;
  if ((footPositions.front()).onFloor[LEFT_FOOT])
  {
    step=Pose3f(RotationMatrix(0, 0, -(footPositions.front()).footPos[LEFT_FOOT].r),
      Vector3f((footPositions.front()).footPos[LEFT_FOOT].x*1000, (footPositions.front()).footPos[LEFT_FOOT].y*1000, 0));
    measuredCoM = Pose3f(theRobotModel.limbs[Limbs::footLeft].inverse()) * measuredCoM;
  }
  else
  {
    step=Pose3f(RotationMatrix(0, 0, -(footPositions.front()).footPos[RIGHT_FOOT].r),
      Vector3f((footPositions.front()).footPos[RIGHT_FOOT].x*1000, (footPositions.front()).footPos[RIGHT_FOOT].y*1000, 0));
    measuredCoM = Pose3f(theRobotModel.limbs[Limbs::footRight].inverse()) * measuredCoM;

  }
  measuredCoM.translate(0, 0, 46);// Use RobotDimenions instead
  footPos = step * (measuredCoM); 
  (Point &)theActualCoM = Point(footPos.translation.x()/1000, footPos.translation.y()/1000, footPos.translation.z()/1000);

  // Delete old foot step
  footPositions.pop_front();
}
