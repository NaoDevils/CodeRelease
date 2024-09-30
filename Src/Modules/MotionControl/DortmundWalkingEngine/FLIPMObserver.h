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
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/FLIPMObservedState.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/MotionControl/FootSteps.h"
#define COMMA ,

MODULE(FLIPMObserver,
  REQUIRES(WalkCalibration),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  REQUIRES(FLIPMObservedState),
  REQUIRES(ZMPModel),
  REQUIRES(FootSteps),
  REQUIRES(FLIPMParameter),
  REQUIRES(FLIPMObserverParameter),
  USES(WalkingInfo),
  USES(TargetCoM),
  PROVIDES(ObservedFLIPMError)
);

class FLIPMObserver : public FLIPMObserverBase
{
public:
  FLIPMObserver() : counter(0), localSensorScale(0), filteredAccX(0.0), filteredAccY(0.0){};

  void update(ObservedFLIPMError& observedFLIPMError);

private:
  RingBuffer<Vector2f, PREVIEW_LENGTH> accDelayBuffer, /**< Buffer to deal with the sensor delay. */
      coM1DelayBuffer, /**< Buffer to deal with the sensor delay. */
      coM2DelayBuffer, /**< Buffer to deal with the sensor delay. */
      realCoM1DelayBuffer, realCoM2DelayBuffer;
  int counter;
  float localSensorScale;
  Point lastRealZMP;

  float filteredAccX;
  float filteredAccY;
};
