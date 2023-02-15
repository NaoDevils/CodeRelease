#include "CoMProvider.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Sensing/RobotModel.h"

using namespace std;

CoMProvider::CoMProvider(
    //const JointAngles				&theJointAngles,
    //const WalkingEngineParams	&theWalkingEngineParams,
    //const JointRequest			&theJointRequest,
    const WalkingInfo& theWalkingInfo,
    //const FootSteps				&theFootSteps,
    //const RobotModel       &theRobotModel,
    const ActualCoMRCS& theActualCoMRCS)
    : //theJointAngles(theJointAngles), all unused
      //theWalkingEngineParams(theWalkingEngineParams),
      //theJointRequest(theJointRequest),
      theWalkingInfo(theWalkingInfo),
      //theFootSteps(theFootSteps),
      //Rensen: Removed due to unused warning
      //theRobotModel(theRobotModel),
      theActualCoMRCS(theActualCoMRCS)
{
}


CoMProvider::~CoMProvider(void) {}


void CoMProvider::updateActualCoM(ActualCoM& theActualCoM)
{
  /*for (int i = 0; i < theFootSteps.getNumOfSteps(); i++)
		footPositions.push_back(theFootSteps.getStep(i));
	if (footPositions.empty()) {
		(Point &)theActualCoM = Point(0.0, 0.0);
		return;
	}
	static bool running = false;
	if (!theFootSteps.running && running)
	{
		footPositions.clear();
		running = false;
		(Point &)theActualCoM = Point(0.0, 0.0);
		return;
	}
	else if (theFootSteps.running && !running)
	{
		running = true;
	}*/

  //Pose2f measuredCoM(Vector2f(theActualCoMRCS.x, theActualCoMRCS.y));
  Point measuredCoM(theActualCoMRCS.x, theActualCoMRCS.y, theActualCoMRCS.z);
  //Pose3f step;
  //Pose3f footPos;
  /* if ((footPositions.front()).onFloor[LEFT_FOOT])
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

	}*/
  (Point&)theActualCoM = theWalkingInfo.toWorldCoords(measuredCoM);

  // Delete old foot step
  //footPositions.pop_front();
}
