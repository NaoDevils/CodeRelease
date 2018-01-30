#pragma once

#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/ObservedError.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Point.h"
#include "Tools/RingBuffer.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/ActualCoM.h"

#define MAX_SENSOR_DELAY 100

class ZMPIPObserver2012
{
public:
  ZMPIPObserver2012(
    const ControllerParams &theControllerParams,
    const TargetCoM            &theTargetCoM,
    const PatternGenRequest	   &thePatternGenRequest,
    const ZMPModel             &theZMPModel,
    const WalkingInfo          &theWalkingInfo,
    const WalkingEngineParams  &theWalkingEngineParams,
    const ActualCoM            &theActualCoM) : 
    theControllerParams(theControllerParams),
    theTargetCoM(theTargetCoM),
    theWalkingEngineParams(theWalkingEngineParams),
    thePatternGenRequest(thePatternGenRequest),
    theZMPModel(theZMPModel),
    theWalkingInfo(theWalkingInfo),
    theActualCoM(theActualCoM),
    isStable(true),
    //sensorScale(0), unused
    localSensorScale(0),
    //Rensen: Removed due to unused warning
    //stableCounter(0),
    wasOn(false)
    {
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.x[0]");
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.x[1]");
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.x[2]");
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.y[0]");
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.y[1]");
      DECLARE_PLOT("module:ZMPIPObserver2012:observedError.y[2]");}; 

  void update(ObservedError &observedError);
protected:
private:
  const ControllerParams &theControllerParams;
  const TargetCoM            &theTargetCoM;
  const WalkingEngineParams  &theWalkingEngineParams;
  const PatternGenRequest	   &thePatternGenRequest;
  const ZMPModel             &theZMPModel;
  const WalkingInfo          &theWalkingInfo;
  const ActualCoM            &theActualCoM;

  RingBuffer<Vector2f, MAX_SENSOR_DELAY> delayBuffer,	/**< Buffer to deal with the sensor delay. */
    coMDelayBuffer;									/**< Buffer to deal with the sensor delay. */
	
  bool isStable;
  //float sensorScale;		/**< Scaling factor for the measured ZMP error. */ unused
  float localSensorScale;

  // Rensen: Removed due to unused warning
//  int stableCounter;

  Point lastRealZMP;
  bool wasOn;
};

