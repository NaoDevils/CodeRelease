/**
* @file FLIPMObserver.h
* @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
*/

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/RobotParameters.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Representations/MotionControl/FLIPMControllerParams.h"
#include "Representations/MotionControl/FLIPMObserverParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/Footpositions.h"

#define COMMA ,

MODULE(FLIPMObserver,
{ ,
  REQUIRES(WalkingEngineParams),
  REQUIRES(ActualCoM),
  REQUIRES(ActualCoMFLIPM),
  REQUIRES(FLIPMControllerParams),
  REQUIRES(FLIPMObserverParams),
  REQUIRES(InertialSensorData),
  REQUIRES(ZMPModel),
  REQUIRES(FootSteps),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(Footpositions),
  USES(WalkingInfo),
  USES(TargetCoM),
  PROVIDES(ObservedFLIPMError),
});

class FLIPMObserver : public FLIPMObserverBase
{

public:
  FLIPMObserver() :
    counter(0),
    isStable(true),
    localSensorScale(0),
    filteredAccX(0.0),
    filteredAccY(0.0)
  {};

  void update(ObservedFLIPMError &observedFLIPMError);

private:

  RingBuffer<Vector2f, PREVIEW_LENGTH> accDelayBuffer,	/**< Buffer to deal with the sensor delay. */
    coM1DelayBuffer,									/**< Buffer to deal with the sensor delay. */
    coM2DelayBuffer,									/**< Buffer to deal with the sensor delay. */
    realCoM1DelayBuffer,
    realCoM2DelayBuffer;
  int counter;
  bool isStable;
  float localSensorScale;
  Point lastRealZMP;

  float filteredAccX;
  float filteredAccY;
};

