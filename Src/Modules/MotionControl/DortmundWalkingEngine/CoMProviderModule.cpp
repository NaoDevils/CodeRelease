/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CoMProviderModule.cpp
* Module wrapper for the CoMProvider
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "CoMProviderModule.h"
#include "Tools/Debugging/DebugDrawings.h"

CoMProviderModule::CoMProviderModule():
controller(
           //theJointAngles,
           //theWalkingEngineParams,
           //theJointRequest,
           theWalkingInfo,
           //theFootSteps,
           //theRobotModel,
           theActualCoMRCS)
{
}

void CoMProviderModule::update(ActualCoM &theActualCoM)
{
  controller.updateActualCoM(theActualCoM);
  //PLOT("module:CoMProvider:JointData:LHipRoll", theJointAngles.angles[Joints::lHipRoll]);
  //PLOT("module:CoMProvider:JointData:LHipPitch", theJointAngles.angles[Joints::lHipPitch]);
  //PLOT("module:CoMProvider:JointData:LKneePitch", theJointAngles.angles[Joints::lKneePitch]);
  //PLOT("module:CoMProvider:JointData:LAnklePitch", theJointAngles.angles[Joints::lAnklePitch]);
  //PLOT("module:CoMProvider:JointData:LAnkleRoll", theJointAngles.angles[Joints::lAnkleRoll]);
  //PLOT("module:CoMProvider:JointData:LHipYawPitch", theJointAngles.angles[Joints::lHipYawPitch]);
  //
  //PLOT("module:CoMProvider:JointData:RHipRoll", theJointAngles.angles[Joints::rHipRoll]);
  //PLOT("module:CoMProvider:JointData:RHipPitch", theJointAngles.angles[Joints::rHipPitch]);
  //PLOT("module:CoMProvider:JointData:RKneePitch", theJointAngles.angles[Joints::rKneePitch]);
  //PLOT("module:CoMProvider:JointData:RAnklePitch", theJointAngles.angles[Joints::rAnklePitch]);
  //PLOT("module:CoMProvider:JointData:RAnkleRoll", theJointAngles.angles[Joints::rAnkleRoll]);
  //PLOT("module:CoMProvider:JointData:RHipYawPitch", theJointAngles.angles[Joints::rHipYawPitch]);
  //PLOT("module:CoMProvider:ActualCoM.x", theActualCoM.x);
  //PLOT("module:CoMProvider:ActualCoM.y", theActualCoM.y);
};


MAKE_MODULE(CoMProviderModule, dortmundWalkingEngine)