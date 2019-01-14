#include "FLIPMObserver.h"
//#include "CSTransform.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

void FLIPMObserver::update(ObservedFLIPMError& observedFLIPMError)
{
  //////////////////////////////////////////////////////
  DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM1.x");	/////////////////
  DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM1.y");	//// Targets ////
  DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM2.x");	/////////////////
  DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM2.y");
  DECLARE_PLOT("module:FLIPMObserver:Delayed_ZMP.x");
  DECLARE_PLOT("module:FLIPMObserver:Delayed_ZMP.y");
  DECLARE_PLOT("module:FLIPMObserver:Delayed_Acc.x");
  DECLARE_PLOT("module:FLIPMObserver:Delayed_Acc.y");
  //////////////////////////////////////////////////////
  DECLARE_PLOT("module:FLIPMObserver:realCoM1.x");		/////////////////////
  DECLARE_PLOT("module:FLIPMObserver:realCoM1.y");		//// Real Values ////
  DECLARE_PLOT("module:FLIPMObserver:realZMP.x");			/////////////////////
  DECLARE_PLOT("module:FLIPMObserver:realZMP.y");
  DECLARE_PLOT("module:FLIPMObserver:realAcc.x");
  DECLARE_PLOT("module:FLIPMObserver:realAcc.y");
  DECLARE_PLOT("module:FLIPMObserver:realCoM2.x");
  DECLARE_PLOT("module:FLIPMObserver:realCoM2.y");
  //////////////////////////////////////////////////////
  DECLARE_PLOT("module:FLIPMObserver:ACC1diff.x");		   /////////////////////
  DECLARE_PLOT("module:FLIPMObserver:ACC1diff.y");		   //// Differences ////
  DECLARE_PLOT("module:FLIPMObserver:CoM1diff.x");		   /////////////////////
  DECLARE_PLOT("module:FLIPMObserver:CoM1diff.y");
  DECLARE_PLOT("module:FLIPMObserver:ZMP1diff.x");
  DECLARE_PLOT("module:FLIPMObserver:ZMP1diff.y");
  DECLARE_PLOT("module:FLIPMObserver:CoM2diff.x");
  DECLARE_PLOT("module:FLIPMObserver:CoM2diff.y");
  //////////////////////////////////////////////////////
  DECLARE_PLOT("module:FLIPMObserver:CoM1diffRatio.x");	  ///////////////
  DECLARE_PLOT("module:FLIPMObserver:CoM1diffRatio.y");	  //// Ratio ////
  DECLARE_PLOT("module:FLIPMObserver:ACC1diffRatio.x");	  ///////////////
  DECLARE_PLOT("module:FLIPMObserver:ACC1diffRatio.y");
  DECLARE_PLOT("module:FLIPMObserver:CoM2diffRatio.x");
  DECLARE_PLOT("module:FLIPMObserver:CoM2diffRatio.y");
  //////////////////////////////////////////////////////

  static int delayCounter = 0;

  /* retrieve robotPosition */
  Pose2f robotPosition(theWalkingInfo.robotPosition.rotation, Vector2f(theWalkingInfo.robotPosition.translation.x(), theWalkingInfo.robotPosition.translation.y()));

  ////////////////
  //// Offset ////
  ////////////////

  /* Offset (RCS)*/
  // TODO: unused!
  //float yOffset = theWalkingEngineParams.comOffsets.ySpeedDependent[theFootpositions.speed.y < 0] * 1000.0f * std::fabs(theFootpositions.speed.y * 1000.0f) / theWalkingEngineParams.speedLimits.y;
  //Vector2f Offset(-theWalkingEngineParams.comOffsets.xFixed, 0.0f - theWalkingEngineParams.comOffsets.yFixed - yOffset / 1000.0f);
  Vector2f Offset(0.f, 0.0f);

  /* rotate Offset from RCS in WCS */
  Offset.rotate(robotPosition.rotation);

  PLOT("module:FLIPMObserver:Offset.x", Offset.x());
  PLOT("module:FLIPMObserver:Offset.y", Offset.y());

  /////////////////
  //// Targets ////
  /////////////////

  /* retrieve target values (WCS) */
  Vector2f targetAccWCS(theTargetCoM.state_x[2], theTargetCoM.state_y[2]);
  Vector2f targetCoM1WCS(theTargetCoM.state_x[0] + Offset.x(), theTargetCoM.state_y[0] + Offset.y());
  Vector2f targetCoM2WCS(theTargetCoM.state_x[3] + Offset.x(), theTargetCoM.state_y[3] + Offset.y());

  /* rotate target values from WCS in RCS */
  if (theFLIPMObserverParams.useRCS) {
    targetAccWCS.rotate(-robotPosition.rotation);
    targetCoM1WCS.rotate(-robotPosition.rotation);
    targetCoM2WCS.rotate(-robotPosition.rotation);
  }

  /* add the target values on the delay buffer */
  accDelayBuffer.push_front(targetAccWCS);
  coM1DelayBuffer.push_front(targetCoM1WCS);
  coM2DelayBuffer.push_front(targetCoM2WCS);

  ///////////////
  //// Reals ////
  ///////////////

  /* retrieve real values */
  Vector2f realAcc, realCoM1, realCoM2;

  //////////////
  //// CoM1 ////
  //////////////

  /* CoM1 Provider (WCS) */
  if (theFLIPMObserverParams.CoM1Provider == FLIPMObserverParams::MRE_CoM) {
    realCoM1.x() = theActualCoMFLIPM.x;
    realCoM1.y() = theActualCoMFLIPM.y;
  }
  else if (theFLIPMObserverParams.CoM1Provider == FLIPMObserverParams::targetCoM) {
    realCoM1.x() = coM1DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].x();
    realCoM1.y() = coM1DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].y();
  }
  else if (theFLIPMObserverParams.CoM1Provider == FLIPMObserverParams::IMU_CoM) {
    realCoM1.x() = theActualCoM.x;
    realCoM1.y() = theActualCoM.y;
  }

  /* add offsets to CoM1 which are in WCS */
  //realCoM1.x() += Offset.x();
  //realCoM1.y() += Offset.y();

  /* CoM1 Provider (WCS->RCS) */
  realCoM1.rotate(-robotPosition.rotation);

  /* add offsets to CoM1 which are in RCS */
  realCoM1.x() += theFLIPMObserverParams.CoM1OffsetX;
  realCoM1.y() += theFLIPMObserverParams.CoM1OffsetY;

  /* CoM1 Provider (RCS->WCS)*/
  if (!theFLIPMObserverParams.useRCS) {
    realCoM1.rotate(robotPosition.rotation);
  }

  realCoM1DelayBuffer.push_front(realCoM1);

  //////////////
  //// CoM2 ////
  //////////////

  /* CoM2 Provider (WCS) */
  if (theFLIPMObserverParams.CoM2Provider == FLIPMObserverParams::MRE_CoM) {
    realCoM2.x() = theActualCoMFLIPM.x;
    realCoM2.y() = theActualCoMFLIPM.y;
  }
  else if (theFLIPMObserverParams.CoM2Provider == FLIPMObserverParams::targetCoM) {
    realCoM2.x() = coM2DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].x();
    realCoM2.y() = coM2DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].y();
  }
  else if (theFLIPMObserverParams.CoM2Provider == FLIPMObserverParams::IMU_CoM) {
    realCoM2.x() = theActualCoM.x;
    realCoM2.y() = theActualCoM.y;
  }

  /* add offsets to CoM2 which are in WCS */
  //realCoM2.x() += Offset.x();
  //realCoM2.y() += Offset.y();

  /* CoM2 Provider (WCS->RCS) */
  realCoM2.rotate(-robotPosition.rotation);

  /* add offsets to CoM2 which are in RCS */
  realCoM2.x() += theFLIPMObserverParams.CoM2OffsetX;
  realCoM2.y() += theFLIPMObserverParams.CoM2OffsetY;

  /* CoM2 Provider (RCS->WCS)*/
  if (!theFLIPMObserverParams.useRCS) {
    realCoM2.rotate(robotPosition.rotation);
  }

  realCoM2DelayBuffer.push_front(realCoM2);

  //////////////
  //// ACC1 ////
  //////////////

  /* ACC Provider (WCS)*/
  if (theFLIPMObserverParams.ACC1Provider == FLIPMObserverParams::ZMP_CoM1_Acc) {
    realAcc.x() = (-1 * theZMPModel.ZMP_WCS.x() + realCoM1.x()) * (theFLIPMControllerParams.paramsX.g / theFLIPMControllerParams.paramsX.z_h);
    realAcc.y() = (-1 * theZMPModel.ZMP_WCS.y() + realCoM1.y()) * (theFLIPMControllerParams.paramsY.g / theFLIPMControllerParams.paramsY.z_h);
  }
  else if (theFLIPMObserverParams.ACC1Provider == FLIPMObserverParams::ZMP_Acc) {
    float filter = theFLIPMObserverParams.accFilter;
    filteredAccX = filteredAccX* (1 - filter) + (-1 * theZMPModel.ZMP_WCS.x() + realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay].x()) * (theFLIPMControllerParams.paramsX.g / theFLIPMControllerParams.paramsX.z_h) * filter;
    filteredAccY = filteredAccY* (1 - filter) + (-1 * theZMPModel.ZMP_WCS.y() + realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay].y()) * (theFLIPMControllerParams.paramsY.g / theFLIPMControllerParams.paramsY.z_h) * filter;
    realAcc.x() = filteredAccX;
    realAcc.y() = filteredAccY;
  }
  else if (theFLIPMObserverParams.ACC1Provider == FLIPMObserverParams::targetAcc) {
    realAcc.x() = accDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].x();
    realAcc.y() = accDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].y();
  }
  else if (theFLIPMObserverParams.ACC1Provider == FLIPMObserverParams::IMU_Acc) {
    float filter = theFLIPMObserverParams.accFilter;
    filteredAccX = filteredAccX* (1 - filter) + (theInertialSensorData.acc.x()) * filter;
    filteredAccY = filteredAccY* (1 - filter) + (theInertialSensorData.acc.y()) * filter;
    realAcc.x() = filteredAccX;
    realAcc.y() = filteredAccY;
  }

  /* ACC Provider (WCS->RCS) */
  if (theFLIPMObserverParams.useRCS) {
    realAcc.rotate(-robotPosition.rotation);
  }

  /////////////////////
  //// Differences ////
  /////////////////////

  /* calculate the CoM differences and the Acceleration difference */
  Vector2f ACC_1_diff, CoM_1_diff, CoM_2_diff;

  if (theTargetCoM.isRunning
    && static_cast<unsigned int>(accDelayBuffer.size()) > theWalkingEngineParams.sensorControl.sensorDelay
    && static_cast<unsigned int>(coM1DelayBuffer.size()) > theWalkingEngineParams.sensorControl.halSensorDelay
    && static_cast<unsigned int>(realCoM2DelayBuffer.size()) > theFLIPMObserverParams.CoM2Delay
    && static_cast<unsigned int>(realCoM1DelayBuffer.size()) > theFLIPMObserverParams.CoM1Delay)
  {
    /* ZMP calculation
    Vector2f ZMPDelay, realZMP;
    ZMPDelay.x() = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].x() - (accDelayBuffer[theWalkingEngineParams.sensorDelay].x()*(paramsFLIPMObsv.z_h / paramsFLIPMObsv.g));
    ZMPDelay.y() = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].y() - (accDelayBuffer[theWalkingEngineParams.sensorDelay].y()*(paramsFLIPMObsv.z_h / paramsFLIPMObsv.g));
    PLOT("module:FLIPMObserver:Delayed_ZMP.x", ZMPDelay.x());
    PLOT("module:FLIPMObserver:Delayed_ZMP.y", ZMPDelay.y());

    realZMP.x() = theZMPModel.ZMP_WCS.x();
    realZMP.y() = theZMPModel.ZMP_WCS.y();
    PLOT("module:FLIPMObserver:realZMP.x", realZMP.x());
    PLOT("module:FLIPMObserver:realZMP.y", realZMP.y());
    ZMP_1_diff = realZMP - ZMPDelay;
    */

    ACC_1_diff = realAcc - accDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay];
    CoM_1_diff = realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay] - coM1DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay];
    CoM_2_diff = realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay] - realCoM2DelayBuffer[theFLIPMObserverParams.CoM2Delay] + Vector2f(theFLIPMObserverParams.CoM2DiffXOffset, theFLIPMObserverParams.CoM2DiffYOffset);

    ACC_1_diff.x() = ACC_1_diff.x() > 0 ? std::min(ACC_1_diff.x(), theFLIPMObserverParams.maxDiffACCClip) : std::max(ACC_1_diff.x(), -theFLIPMObserverParams.maxDiffACCClip);
    ACC_1_diff.y() = ACC_1_diff.y() > 0 ? std::min(ACC_1_diff.y(), theFLIPMObserverParams.maxDiffACCClip) : std::max(ACC_1_diff.y(), -theFLIPMObserverParams.maxDiffACCClip);

    CoM_1_diff.x() = CoM_1_diff.x() > 0 ? std::min(CoM_1_diff.x(), theFLIPMObserverParams.maxDiffCoMClip) : std::max(CoM_1_diff.x(), -theFLIPMObserverParams.maxDiffCoMClip);
    CoM_1_diff.y() = CoM_1_diff.y() > 0 ? std::min(CoM_1_diff.y(), theFLIPMObserverParams.maxDiffCoMClip) : std::max(CoM_1_diff.y(), -theFLIPMObserverParams.maxDiffCoMClip);

    CoM_2_diff.x() = CoM_2_diff.x() > 0 ? std::min(CoM_2_diff.x(), theFLIPMObserverParams.maxDiffCoMClip) : std::max(CoM_2_diff.x(), -theFLIPMObserverParams.maxDiffCoMClip);
    CoM_2_diff.y() = CoM_2_diff.y() > 0 ? std::min(CoM_2_diff.y(), theFLIPMObserverParams.maxDiffCoMClip) : std::max(CoM_2_diff.y(), -theFLIPMObserverParams.maxDiffCoMClip);
  }
  else {
    ACC_1_diff.setZero();
    CoM_1_diff.setZero();
    CoM_2_diff.setZero();
    filteredAccX = 0.0;
    filteredAccY = 0.0;
  }

  ///////////////
  // Targets ////
  ///////////////
  PLOT("module:FLIPMObserver:Delayed_CoM1.x", coM1DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].x());
  PLOT("module:FLIPMObserver:Delayed_CoM1.y", coM1DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].y());
  PLOT("module:FLIPMObserver:Delayed_CoM2.x", coM2DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].x());
  PLOT("module:FLIPMObserver:Delayed_CoM2.y", coM2DelayBuffer[theWalkingEngineParams.sensorControl.halSensorDelay].y());
  PLOT("module:FLIPMObserver:Delayed_Acc.x", accDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].x());
  PLOT("module:FLIPMObserver:Delayed_Acc.y", accDelayBuffer[theWalkingEngineParams.sensorControl.sensorDelay].y());
  ///////////////////
  // Real Values ////
  ///////////////////
  PLOT("module:FLIPMObserver:realCoM1.x", realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay].x());
  PLOT("module:FLIPMObserver:realCoM1.y", realCoM1DelayBuffer[theFLIPMObserverParams.CoM1Delay].y());
  PLOT("module:FLIPMObserver:realAcc.x", realAcc.x());
  PLOT("module:FLIPMObserver:realAcc.y", realAcc.y());
  PLOT("module:FLIPMObserver:realCoM2.x", realCoM2DelayBuffer[theFLIPMObserverParams.CoM2Delay].x());
  PLOT("module:FLIPMObserver:realCoM2.y", realCoM2DelayBuffer[theFLIPMObserverParams.CoM2Delay].y());
  ///////////////////
  // Differences ////
  ///////////////////
  PLOT("module:FLIPMObserver:ACC1diff.x", ACC_1_diff.x());
  PLOT("module:FLIPMObserver:ACC1diff.y", ACC_1_diff.y());
  PLOT("module:FLIPMObserver:CoM1diff.x", CoM_1_diff.x());
  PLOT("module:FLIPMObserver:CoM1diff.y", CoM_1_diff.y());
  PLOT("module:FLIPMObserver:CoM2diff.x", CoM_2_diff.x());
  PLOT("module:FLIPMObserver:CoM2diff.y", CoM_2_diff.y());

  /* calculate if walking or not!*/
#if 1
  static bool sensorOn = false;

  if (theFootSteps.walkState == State::walking &&
    isStable)
    sensorOn = true;

  if (!(theFootSteps.walkState == State::walking))
    sensorOn = false;

#else
  static bool sensorOn = true;
#endif

  /* ZMP Smoothing to reduce jerk*/
  if (sensorOn && localSensorScale < 1) {
    if (counter > 25) {
      localSensorScale += 1.0f / theWalkingEngineParams.walkTransition.zmpSmoothPhase;
    }
    else {
      counter++;
    }
  }
  if (!sensorOn) {
    delayCounter++;
    counter = 0;
  }
  else {
    delayCounter = 0;
  }
  if (delayCounter > theFLIPMControllerParams.paramsX.N && localSensorScale > 0) {
    localSensorScale -= 1.0f / theWalkingEngineParams.walkTransition.zmpSmoothPhase;
  }

  if (theFLIPMObserverParams.useRCS) {
    CoM_1_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[0] * localSensorScale);
    CoM_1_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[0] * localSensorScale);
    ACC_1_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[1] * localSensorScale);
    ACC_1_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[1] * localSensorScale);
    CoM_2_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[2] * localSensorScale);
    CoM_2_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[2] * localSensorScale);
  }
  else {
    /* Transform WCS into RCS and back, to apply some scaling*/
    CoM_1_diff.rotate(-robotPosition.rotation);
    CoM_1_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[0] * localSensorScale) *theFLIPMObserverParams.sensorXFactor;
    CoM_1_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[0] * localSensorScale) *theFLIPMObserverParams.sensorYFactor;
    CoM_1_diff.rotate(robotPosition.rotation);

    ACC_1_diff.rotate(-robotPosition.rotation);
    ACC_1_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[1] * localSensorScale) *theFLIPMObserverParams.sensorXFactor;
    ACC_1_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[1] * localSensorScale) *theFLIPMObserverParams.sensorYFactor;
    ACC_1_diff.rotate(robotPosition.rotation);

    CoM_2_diff.rotate(-robotPosition.rotation);
    CoM_2_diff.x() *= (theFLIPMObserverParams.sensorControlRatioObserverX[2] * localSensorScale) *theFLIPMObserverParams.sensorXFactor;
    CoM_2_diff.y() *= (theFLIPMObserverParams.sensorControlRatioObserverY[2] * localSensorScale) *theFLIPMObserverParams.sensorYFactor;
    CoM_2_diff.rotate(robotPosition.rotation);
  }

  MODIFY("module:FLIPMObserver:CoM_1_diff", CoM_1_diff);
  MODIFY("module:FLIPMObserver:ACC_1_diff", ACC_1_diff);
  MODIFY("module:FLIPMObserver:CoM_2_diff", CoM_2_diff);

  if (theFLIPMObserverParams.activateSensorX == true) {
    observedFLIPMError.ObservedError.x() = theFLIPMControllerParams.paramsX.L * Vector3f(CoM_1_diff.x(), ACC_1_diff.x(), CoM_2_diff.x());
  }
  else {
    observedFLIPMError.ObservedError.x() = theFLIPMControllerParams.paramsX.L * Vector3f(0, 0, 0);
  }

  if (theFLIPMObserverParams.activateSensorY == true) {
    observedFLIPMError.ObservedError.y() = theFLIPMControllerParams.paramsY.L * Vector3f(CoM_1_diff.y(), ACC_1_diff.y(), CoM_2_diff.y());
  }
  else {
    observedFLIPMError.ObservedError.y() = theFLIPMControllerParams.paramsY.L * Vector3f(0, 0, 0);
  }

  PLOT("module:FLIPMObserver:ACC1diffRatio.x", ACC_1_diff.x());
  PLOT("module:FLIPMObserver:ACC1diffRatio.y", ACC_1_diff.y());
  PLOT("module:FLIPMObserver:CoM1diffRatio.x", CoM_1_diff.x());
  PLOT("module:FLIPMObserver:CoM1diffRatio.y", CoM_1_diff.y());
  PLOT("module:FLIPMObserver:CoM2diffRatio.x", CoM_2_diff.x());
  PLOT("module:FLIPMObserver:CoM2diffRatio.y", CoM_2_diff.y());
  LOG("FLIPMObserver", "ACC1diffRatio.x", ACC_1_diff.x());
  LOG("FLIPMObserver", "ACC1diffRatio.y", ACC_1_diff.y());
  LOG("FLIPMObserver", "CoM1diffRatio.x", CoM_1_diff.x());
  LOG("FLIPMObserver", "CoM1diffRatio.y", CoM_1_diff.y());
  LOG("FLIPMObserver", "CoM2diffRatio.x", CoM_2_diff.x());
  LOG("FLIPMObserver", "CoM2diffRatio.y", CoM_2_diff.y());

  LOG("FLIPMObserver", "angleX", theInertialSensorData.angle.x());
  LOG("FLIPMObserver", "angleY", theInertialSensorData.angle.x());
  if (theTargetCoM.isRunning)
  {
    LOG("WalkingEngine", "real Acc x", realAcc.x());
    LOG("WalkingEngine", "real Acc y", realAcc.y());
  }

}


MAKE_MODULE(FLIPMObserver, dortmundWalkingEngine)