#include "ZMPIPObserver2012.h"
//#define LOGGING
//#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Debugging/Modify.h"

void ZMPIPObserver2012::update(ObservedError& observedError)
{
  static int delayCounter=0;
  
  if (wasOn && !theTargetCoM.isRunning)
  {
    wasOn = false;
    delayBuffer.clear();
    coMDelayBuffer.clear();
  }
  if (!wasOn && theTargetCoM.isRunning)
  {
    wasOn = true;
  }
  
  Vector2f xOffset(theWalkingEngineParams.xOffset, 0);
  xOffset.rotate(theWalkingInfo.robotPosition.rotation);

  Vector2f target(theTargetCoM.state_x[2] + xOffset.x(),
    theTargetCoM.state_y[2] + xOffset.y());
  Vector2f targetCoM(theTargetCoM.state_x[0] + xOffset.x(),
    theTargetCoM.state_y[0] + xOffset.y());

  delayBuffer.push_front(target);
  coMDelayBuffer.push_front(targetCoM);
  
  Vector2f realZMP(theZMPModel.ZMP_WCS.x(), theZMPModel.ZMP_WCS.y());
  Vector2f realCoM(theActualCoM.x, theActualCoM.y);
  Pose2f robotPosition(theWalkingInfo.robotPosition.rotation,
                         Vector2f(theWalkingInfo.robotPosition.translation.x(),
                                   theWalkingInfo.robotPosition.translation.y()));

  Vector2f ZMPdiff, CoMdiff;
  if (delayBuffer.size()>theWalkingEngineParams.sensorDelay)
  {
    ZMPdiff = realZMP - delayBuffer[theWalkingEngineParams.sensorDelay];
    CoMdiff = realCoM - coMDelayBuffer[theWalkingEngineParams.halSensorDelay];
  }
  else
  {
    ZMPdiff.setZero();
    CoMdiff.setZero();
  }

#if 1
  static bool sensorOn=false;

  if ((thePatternGenRequest.newState==PatternGenRequest::walking ||
	  theWalkingInfo.kickPhase!=freeLegNA) &&
    isStable)
	  sensorOn=true;

  if (!(thePatternGenRequest.newState==PatternGenRequest::walking ||
	  theWalkingInfo.kickPhase!=freeLegNA))
	  sensorOn=false;
#else
  static bool sensorOn=true;
#endif

  if (sensorOn && localSensorScale<1)
	  localSensorScale+=1.0f/theWalkingEngineParams.zmpSmoothPhase;

  if (!sensorOn)
	  delayCounter++;
  else
	  delayCounter=0;

  if (delayCounter>theControllerParams.N && localSensorScale>0)
	  localSensorScale-=1.0f/theWalkingEngineParams.zmpSmoothPhase;

  ZMPdiff.rotate(-robotPosition.rotation);
  ZMPdiff *= (theWalkingEngineParams.sensorControlRatio[1] * localSensorScale);
  ZMPdiff.rotate(robotPosition.rotation);

  CoMdiff.rotate(-robotPosition.rotation);
  CoMdiff *= (localSensorScale * theWalkingEngineParams.sensorControlRatio[0]);
  CoMdiff.rotate(robotPosition.rotation);

  MODIFY("module:ZMPIPObserver:CoMdiff", CoMdiff);
  MODIFY("module:ZMPIPObserver:ZMPdiff", ZMPdiff);
  
  observedError.CoM_WCS.x() = theControllerParams.L * Vector2f(CoMdiff.x(), 0);
  observedError.ZMP_WCS.x() = theControllerParams.L * Vector2f(0, ZMPdiff.x());
  observedError.CoM_WCS.y() = theControllerParams.L * Vector2f(CoMdiff.y(), 0);
  observedError.ZMP_WCS.y() = theControllerParams.L * Vector2f(0, ZMPdiff.y());

  PLOT("module:ZMPIPObserver2012:Delayed_CoM.x", coMDelayBuffer[theWalkingEngineParams.halSensorDelay].x());
  PLOT("module:ZMPIPObserver2012:Delayed_CoM.y", coMDelayBuffer[theWalkingEngineParams.halSensorDelay].y());
  PLOT("module:ZMPIPObserver2012:Delayed_ZMP.x", delayBuffer[theWalkingEngineParams.sensorDelay].x());
  PLOT("module:ZMPIPObserver2012:Delayed_ZMP.y", delayBuffer[theWalkingEngineParams.sensorDelay].y());
  PLOT("module:ZMPIPObserver2012:ZMPDiff.x", ZMPdiff.x());
  PLOT("module:ZMPIPObserver2012:ZMPDiff.y", ZMPdiff.y());

  /*if (theTargetCoM.isRunning)
  {
    LOG("WalkingEngine", "real ZMP x", realZMP.x);
    LOG("WalkingEngine", "real ZMP y", realZMP.y);
  }*/
}
