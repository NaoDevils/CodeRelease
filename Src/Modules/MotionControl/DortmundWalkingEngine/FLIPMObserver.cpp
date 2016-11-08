#include "FLIPMObserver.h"
//#include "CSTransform.h"

#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

void FLIPMObserver::update(ObservedFLIPMError& observedFLIPMError)
{	
  MODIFY("representation:FLIPMObserverParams", flipmObserverParams);
	//////////////////////////////////////////////////////
	DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM1.x");	/////////////////
	DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM1.y");	//// Targets ////
	DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM2.x");	/////////////////
	DECLARE_PLOT("module:FLIPMObserver:Delayed_CoM2.y");
	DECLARE_PLOT("module:FLIPMObserver:Delayed_ZMP.x");		
	DECLARE_PLOT("module:FLIPMObserver:Delayed_ZMP.y");	
	DECLARE_PLOT("module:FLIPMObserver:Delayed_Acc.x");
	DECLARE_PLOT("module:FLIPMObserver:Delayed_Acc.y");
	MARK("FLIPMObserver", "Delayed_CoM1.x");
	MARK("FLIPMObserver", "Delayed_CoM1.y");
	MARK("FLIPMObserver", "Delayed_CoM2.x");
	MARK("FLIPMObserver", "Delayed_CoM2.y");
	MARK("FLIPMObserver", "Delayed_Acc.x");
	MARK("FLIPMObserver", "Delayed_Acc.y");
	MARK("FLIPMObserver", "Delayed_ZMP.x");
	MARK("FLIPMObserver", "Delayed_ZMP.y");
	//////////////////////////////////////////////////////
	DECLARE_PLOT("module:FLIPMObserver:realCoM1.x");		/////////////////////
	DECLARE_PLOT("module:FLIPMObserver:realCoM1.y");		//// Real Values ////
	DECLARE_PLOT("module:FLIPMObserver:realZMP.x");			/////////////////////
	DECLARE_PLOT("module:FLIPMObserver:realZMP.y");		
	DECLARE_PLOT("module:FLIPMObserver:realAcc.x");		
	DECLARE_PLOT("module:FLIPMObserver:realAcc.y");
	DECLARE_PLOT("module:FLIPMObserver:realCoM2.x");
	DECLARE_PLOT("module:FLIPMObserver:realCoM2.y");
	MARK("FLIPMObserver", "realCoM1.x");
	MARK("FLIPMObserver", "realCoM1.y");
	MARK("FLIPMObserver", "realAcc.x");
	MARK("FLIPMObserver", "realAcc.y");
	MARK("FLIPMObserver", "realZMP.x");
	MARK("FLIPMObserver", "realZMP.y");
	MARK("FLIPMObserver", "realCoM2.x");
	MARK("FLIPMObserver", "realCoM2.y");
	//////////////////////////////////////////////////////
	DECLARE_PLOT("module:FLIPMObserver:ACC1diff.x");		   /////////////////////
	DECLARE_PLOT("module:FLIPMObserver:ACC1diff.y");		   //// Differences ////
	DECLARE_PLOT("module:FLIPMObserver:CoM1diff.x");		   /////////////////////
	DECLARE_PLOT("module:FLIPMObserver:CoM1diff.y");	
	DECLARE_PLOT("module:FLIPMObserver:ZMP1diff.x");
	DECLARE_PLOT("module:FLIPMObserver:ZMP1diff.y");
	DECLARE_PLOT("module:FLIPMObserver:CoM2diff.x");
	DECLARE_PLOT("module:FLIPMObserver:CoM2diff.y");
	MARK("FLIPMObserver", "ACC1diff.x");
	MARK("FLIPMObserver", "ACC1diff.y");
	MARK("FLIPMObserver", "CoM1diff.x");
	MARK("FLIPMObserver", "CoM1diff.y");
	MARK("FLIPMObserver", "ZMP1diff.x");
	MARK("FLIPMObserver", "ZMP1diff.y");
	MARK("FLIPMObserver", "CoM2diff.x");
	MARK("FLIPMObserver", "CoM2diff.y");
	//////////////////////////////////////////////////////
	DECLARE_PLOT("module:FLIPMObserver:CoM1diffRatio.x");	  ///////////////
	DECLARE_PLOT("module:FLIPMObserver:CoM1diffRatio.y");	  //// Ratio ////
	DECLARE_PLOT("module:FLIPMObserver:ACC1diffRatio.x");	  ///////////////
	DECLARE_PLOT("module:FLIPMObserver:ACC1diffRatio.y");
	DECLARE_PLOT("module:FLIPMObserver:CoM2diffRatio.x");
	DECLARE_PLOT("module:FLIPMObserver:CoM2diffRatio.y");
	MARK("FLIPMObserver", "CoM1diffRatio.x");
	MARK("FLIPMObserver", "CoM1diffRatio.y");
	MARK("FLIPMObserver", "ACC1diffRatio.x");
	MARK("FLIPMObserver", "ACC1diffRatio.y");
	MARK("FLIPMObserver", "CoM2diffRatio.x");
	MARK("FLIPMObserver", "CoM2diffRatio.y");
	//////////////////////////////////////////////////////
	MARK("FLIPMObserver", "angleX");
	MARK("FLIPMObserver", "angleY");
  MARK("FLIPMObserver", "battery");
  MARK("FLIPMObserver", "RAnklePitch");
  MARK("FLIPMObserver", "RKneePitch");
  MARK("FLIPMObserver", "RHipPitch");
  MARK("FLIPMObserver", "LAnklePitch");
  MARK("FLIPMObserver", "LKneePitch");
  MARK("FLIPMObserver", "LHipPitch");
	DECLARE_PLOT("module:FLIPMObserver:XOffset.x");
	DECLARE_PLOT("module:FLIPMObserver:XOffset.y");
	DECLARE_PLOT("module:FLIPMObserver:XOffset.abs");


	paramsFLIPMObsvX.handle();
	paramsFLIPMObsvY.handle();

	/* retrieve target values */
	static int delayCounter = 0;
	
  Vector2f XOffset(theWalkingEngineParams.xOffset, 0);
  XOffset.rotate(theWalkingInfo.robotPosition.rotation);

	if (flipmObserverParams.FLIPM_XOffset == 1) {
		XOffset.x() = 0;
		XOffset.y() = 0;
	}

	PLOT("module:FLIPMObserver:XOffset.x", XOffset.x());
	PLOT("module:FLIPMObserver:XOffset.y", XOffset.y());

  Vector2f targetAcc (theTargetCoM.state_x[2], theTargetCoM.state_y[2]);
  Vector2f targetCoM1(theTargetCoM.state_x[0] + XOffset.x(), theTargetCoM.state_y[0] + XOffset.y());
  Vector2f targetCoM2(theTargetCoM.state_x[3] + XOffset.x(), theTargetCoM.state_y[3] + XOffset.y());

	accDelayBuffer.push_front(targetAcc);
	coM1DelayBuffer.push_front(targetCoM1);
	coM2DelayBuffer.push_front(targetCoM2);

  Pose2f robotPosition(theWalkingInfo.robotPosition.rotation,
    Vector2f(theWalkingInfo.robotPosition.translation.x(),
      theWalkingInfo.robotPosition.translation.y()));


	/* retrieve real values */
  Vector2f realAcc, realCoM1, realCoM2;

	/* switch the CoM1 Provider*/
	if (flipmObserverParams.FLIPM_CoM1Provider == 1) {
		realCoM1.x() = theActualCoMFLIPM.x;
		realCoM1.y() = theActualCoMFLIPM.y;
	}
	else if (flipmObserverParams.FLIPM_CoM1Provider == 2) {
    realCoM1 = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay];
	} 	else { // 0 or any other number
		realCoM1.x() = theActualCoM.x;
		realCoM1.y() = theActualCoM.y;
	}

  realCoM1.rotate(-robotPosition.rotation);
  realCoM1.x() += flipmObserverParams.FLIPM_CoM1OffsetX;
  realCoM1.y() += flipmObserverParams.FLIPM_CoM1OffsetY;
  realCoM1.x() *= (flipmObserverParams.FLIPM_CoM1Multiplikator[0]);
  realCoM1.y() *= (flipmObserverParams.FLIPM_CoM1Multiplikator[1]);
  realCoM1.rotate(robotPosition.rotation);

  realCoM1DelayBuffer.push_front(realCoM1);

	/* switch the CoM2 Provider*/
	if (flipmObserverParams.FLIPM_CoM2Provider == 1) {
		realCoM2.x() = theActualCoMFLIPM.x;
		realCoM2.y() = theActualCoMFLIPM.y;
	}
	else if (flipmObserverParams.FLIPM_CoM2Provider == 2) {
		realCoM2 = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay];
	}
	else { // 0 or any other number
		realCoM2.x() = theActualCoM.x;
		realCoM2.y() = theActualCoM.y;
	}

  realCoM2.rotate(-robotPosition.rotation);
  realCoM2.x() += flipmObserverParams.FLIPM_CoM2OffsetX;
  realCoM2.y() += flipmObserverParams.FLIPM_CoM2OffsetY;
  realCoM2.x() *= (flipmObserverParams.FLIPM_CoM2Multiplikator[0]);
  realCoM2.y() *= (flipmObserverParams.FLIPM_CoM2Multiplikator[1]);
  realCoM2.rotate(robotPosition.rotation);

  realCoM2DelayBuffer.push_front(realCoM2);

	/* switch the ZMP Provider*/
	if (flipmObserverParams.FLIPM_ACC1Provider == 1) {
		realAcc.x() = theInertialSensorData.acc.x() * paramsFLIPMObsvX.g;
		realAcc.y() = theInertialSensorData.acc.y() * paramsFLIPMObsvY.g;
	}
	else if (flipmObserverParams.FLIPM_ACC1Provider == 2) {
		realAcc.x() = filteredAccX* flipmObserverParams.FLIPM_IMUFilter + (theInertialSensorData.acc.x() * paramsFLIPMObsvX.g) * (1 - flipmObserverParams.FLIPM_IMUFilter);
		realAcc.y() = filteredAccY* flipmObserverParams.FLIPM_IMUFilter + (theInertialSensorData.acc.y() * paramsFLIPMObsvY.g) * (1 - flipmObserverParams.FLIPM_IMUFilter);
	}
	else { // 0 or any other number
		realAcc.x() = (-1 * theZMPModel.ZMP_WCS.x() + realCoM1.x()) * (paramsFLIPMObsvX.g / paramsFLIPMObsvX.z_h);
		realAcc.y() = (-1 * theZMPModel.ZMP_WCS.y() + realCoM1.y()) * (paramsFLIPMObsvY.g / paramsFLIPMObsvY.z_h);
	}

	/* calculate the ZMP & CoM differences and the Acceleration difference for use with the accelerometer*/
	Vector2f ACC_1_diff, CoM_1_diff, CoM_2_diff, ZMP_1_diff, ZMPDelay, realZMP;
  if (accDelayBuffer.size() > theWalkingEngineParams.sensorDelay && coM1DelayBuffer.size() > theWalkingEngineParams.halSensorDelay && realCoM2DelayBuffer.size() > flipmObserverParams.FLIPM_CoM2Delay)
	{
		ZMPDelay.x() = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].x() - (accDelayBuffer[theWalkingEngineParams.sensorDelay].x()*(paramsFLIPMObsvX.z_h / paramsFLIPMObsvX.g));
		ZMPDelay.y() = coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].y() - (accDelayBuffer[theWalkingEngineParams.sensorDelay].y()*(paramsFLIPMObsvY.z_h / paramsFLIPMObsvY.g));

		realZMP.x() = theZMPModel.ZMP_WCS.x();
		realZMP.y() = theZMPModel.ZMP_WCS.y();

		ZMP_1_diff = realZMP - ZMPDelay;
		ACC_1_diff = realAcc - accDelayBuffer[theWalkingEngineParams.sensorDelay];
    CoM_1_diff = realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay] - coM1DelayBuffer[theWalkingEngineParams.halSensorDelay];

		if (flipmObserverParams.FLIPM_CoM2DiffProvider == 1) {
			CoM_2_diff = realCoM2 - coM2DelayBuffer[theWalkingEngineParams.halSensorDelay];
		}
		else if (flipmObserverParams.FLIPM_CoM2DiffProvider == 2) {
			CoM_2_diff = (realCoM1 - realCoM2) - coM2DelayBuffer[theWalkingEngineParams.halSensorDelay];
		}
		else if (flipmObserverParams.FLIPM_CoM2DiffProvider == 3) {
			CoM_2_diff = (realCoM2 - realCoM1) - coM2DelayBuffer[theWalkingEngineParams.halSensorDelay];
		} 
		else if (flipmObserverParams.FLIPM_CoM2DiffProvider == 4) {
			CoM_2_diff = (realCoM1 - realCoM2) - coM2DelayBuffer[theWalkingEngineParams.halSensorDelay];
		}
		else {
      CoM_2_diff = realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay] - realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay];
		}
		
	}

  realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].x() += flipmObserverParams.FLIPM_CoM1OffsetX;
  realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].x() += flipmObserverParams.FLIPM_CoM2OffsetX;

  realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].y() += flipmObserverParams.FLIPM_CoM1OffsetY;
  realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].y() += flipmObserverParams.FLIPM_CoM2OffsetY;

	///////////////
	// Targets ////
	///////////////
	PLOT("module:FLIPMObserver:Delayed_CoM1.x", coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].x());
	PLOT("module:FLIPMObserver:Delayed_CoM1.y", coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].y());
	PLOT("module:FLIPMObserver:Delayed_CoM2.x", coM2DelayBuffer[theWalkingEngineParams.halSensorDelay].x());
	PLOT("module:FLIPMObserver:Delayed_CoM2.y", coM2DelayBuffer[theWalkingEngineParams.halSensorDelay].y());
	PLOT("module:FLIPMObserver:Delayed_Acc.x", accDelayBuffer[theWalkingEngineParams.sensorDelay].x());
	PLOT("module:FLIPMObserver:Delayed_Acc.y", accDelayBuffer[theWalkingEngineParams.sensorDelay].y());
	PLOT("module:FLIPMObserver:Delayed_ZMP.x", ZMPDelay.x());
	PLOT("module:FLIPMObserver:Delayed_ZMP.y", ZMPDelay.y());
	LOG("FLIPMObserver", "Delayed_CoM1.x", coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].x());
	LOG("FLIPMObserver", "Delayed_CoM1.y", coM1DelayBuffer[theWalkingEngineParams.halSensorDelay].y());
	LOG("FLIPMObserver", "Delayed_CoM2.x", coM2DelayBuffer[theWalkingEngineParams.halSensorDelay].x());
	LOG("FLIPMObserver", "Delayed_CoM2.y", coM2DelayBuffer[theWalkingEngineParams.halSensorDelay].y());
	LOG("FLIPMObserver", "Delayed_Acc.x", accDelayBuffer[theWalkingEngineParams.sensorDelay].x());
	LOG("FLIPMObserver", "Delayed_Acc.y", accDelayBuffer[theWalkingEngineParams.sensorDelay].y());
	LOG("FLIPMObserver", "Delayed_ZMP.x", ZMPDelay.x());
	LOG("FLIPMObserver", "Delayed_ZMP.y", ZMPDelay.y());
	///////////////////
	// Real Values ////
	///////////////////
  PLOT("module:FLIPMObserver:realCoM1.x", realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].x());
  PLOT("module:FLIPMObserver:realCoM1.y", realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].y());
	PLOT("module:FLIPMObserver:realAcc.x", realAcc.x());
	PLOT("module:FLIPMObserver:realAcc.y", realAcc.y());
	PLOT("module:FLIPMObserver:realZMP.x", realZMP.x());
	PLOT("module:FLIPMObserver:realZMP.y", realZMP.y());
  PLOT("module:FLIPMObserver:realCoM2.x", realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].x());
  PLOT("module:FLIPMObserver:realCoM2.y", realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].y());
  LOG("FLIPMObserver", "realCoM1.x", realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].x());
  LOG("FLIPMObserver", "realCoM1.y", realCoM1DelayBuffer[flipmObserverParams.FLIPM_CoM1Delay].y());
	LOG("FLIPMObserver", "realAcc.x", realAcc.x());
	LOG("FLIPMObserver", "realAcc.y", realAcc.y());
	LOG("FLIPMObserver", "realZMP.x", realZMP.x());
	LOG("FLIPMObserver", "realZMP.y", realZMP.y());
  LOG("FLIPMObserver", "realCoM2.x", realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].x());
  LOG("FLIPMObserver", "realCoM2.y", realCoM2DelayBuffer[flipmObserverParams.FLIPM_CoM2Delay].y());
	///////////////////
	// Differences ////
	///////////////////
	PLOT("module:FLIPMObserver:ACC1diff.x", ACC_1_diff.x());
	PLOT("module:FLIPMObserver:ACC1diff.y", ACC_1_diff.y());
	PLOT("module:FLIPMObserver:CoM1diff.x", CoM_1_diff.x());
	PLOT("module:FLIPMObserver:CoM1diff.y", CoM_1_diff.y());
	PLOT("module:FLIPMObserver:ZMP1diff.x", ZMP_1_diff.x());
	PLOT("module:FLIPMObserver:ZMP1diff.y", ZMP_1_diff.y());
	PLOT("module:FLIPMObserver:CoM2diff.x", CoM_2_diff.x());
	PLOT("module:FLIPMObserver:CoM2diff.y", CoM_2_diff.y());
	LOG("FLIPMObserver", "ACC1diff.x", ACC_1_diff.x());
	LOG("FLIPMObserver", "ACC1diff.y", ACC_1_diff.y());
	LOG("FLIPMObserver", "CoM1diff.x", CoM_1_diff.x());
	LOG("FLIPMObserver", "CoM1diff.y", CoM_1_diff.y());
	LOG("FLIPMObserver", "ZMP1diff.x", ZMP_1_diff.x());
	LOG("FLIPMObserver", "ZMP1diff.y", ZMP_1_diff.y());
	LOG("FLIPMObserver", "CoM2diff.x", CoM_2_diff.x());
	LOG("FLIPMObserver", "CoM2diff.y", CoM_2_diff.y());

#if 1
	static bool sensorOn = false;

	if ((thePatternGenRequest.newState == PatternGenRequest::walking ||
		theWalkingInfo.kickPhase != freeLegNA) &&
		isStable)
		sensorOn = true;

	if (!(thePatternGenRequest.newState == PatternGenRequest::walking ||
		theWalkingInfo.kickPhase != freeLegNA))
		sensorOn = false;
#else
	static bool sensorOn = true;
#endif
	/* ZMP Smoothing to reduce jerk*/
	if (sensorOn && localSensorScale<1)
		localSensorScale += 1.0f / theWalkingEngineParams.zmpSmoothPhase;

	if (!sensorOn)
		delayCounter++;
	else
		delayCounter = 0;
	if (delayCounter > paramsFLIPMObsvX.N && localSensorScale>0)

		localSensorScale -= 1.0f / theWalkingEngineParams.zmpSmoothPhase;


	/* Transform WCS into RCS and back, to apply some scaling*/
	CoM_1_diff.rotate(-robotPosition.rotation);
  CoM_1_diff.x() += flipmObserverParams.FLIPM_CoM1DiffOffsetX;
  CoM_1_diff.y() += flipmObserverParams.FLIPM_CoM1DiffOffsetY;
	CoM_1_diff.x() *= (flipmObserverParams.sensorControlRatioObserverX[0] * localSensorScale);
	CoM_1_diff.y() *= (flipmObserverParams.sensorControlRatioObserverY[0] * localSensorScale);
	CoM_1_diff.rotate(robotPosition.rotation);

	ACC_1_diff.rotate(-robotPosition.rotation);
	ACC_1_diff.x() *= (flipmObserverParams.sensorControlRatioObserverX[1] * localSensorScale);
	ACC_1_diff.y() *= (flipmObserverParams.sensorControlRatioObserverY[1] * localSensorScale);
	ACC_1_diff.rotate(robotPosition.rotation);

	CoM_2_diff.rotate(-robotPosition.rotation);
  CoM_2_diff.x() += flipmObserverParams.FLIPM_CoM2DiffOffsetX;
  CoM_2_diff.y() += flipmObserverParams.FLIPM_CoM2DiffOffsetY;
	CoM_2_diff.x() *= (flipmObserverParams.sensorControlRatioObserverX[2] * localSensorScale);
	CoM_2_diff.y() *= (flipmObserverParams.sensorControlRatioObserverY[2] * localSensorScale);
	CoM_2_diff.rotate(robotPosition.rotation);

	MODIFY("module:FLIPMObserver:CoM_1_diff", CoM_1_diff);
	MODIFY("module:FLIPMObserver:ACC_1_diff", ACC_1_diff);
	MODIFY("module:FLIPMObserver:CoM_2_diff", CoM_2_diff);

	observedFLIPMError.CoM_1_WCS.x() = paramsFLIPMObsvX.L *  Vector3f(CoM_1_diff.x(), 0, 0);
	observedFLIPMError.CoM_1_WCS.y() = paramsFLIPMObsvY.L *  Vector3f(CoM_1_diff.y(), 0, 0);
	if (flipmObserverParams.activateSensorX == true) {
    observedFLIPMError.Acc_1_WCS.x() = paramsFLIPMObsvX.L * Vector3f(CoM_1_diff.x(), ACC_1_diff.x(), CoM_2_diff.x());
	}
	else {
		observedFLIPMError.Acc_1_WCS.x() = paramsFLIPMObsvX.L * Vector3f(0, 0, 0);
	}
	
	if (flipmObserverParams.activateSensorY == true) {
		observedFLIPMError.Acc_1_WCS.y() = paramsFLIPMObsvY.L * Vector3f(CoM_1_diff.y(), ACC_1_diff.y(), CoM_2_diff.y());
	}
	else {
		observedFLIPMError.Acc_1_WCS.y() = paramsFLIPMObsvY.L * Vector3f(0,0,0);
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