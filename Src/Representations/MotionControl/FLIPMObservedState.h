/**
* @file ActualAcc
* @author <a href="mailto:arne.moos@tu-dortmund.de>Arne Moos</a>
*/
#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"


struct FLIPMObservedState : public Streamable
{
  void serialize(In* in,Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(actualAcc)
      STREAM(actualCoM_IMU.x)
      STREAM(actualCoM_IMU.y)
      STREAM(actualCoM_IMU.z)
      STREAM(actualCoM_MRE.x)
      STREAM(actualCoM_MRE.y)
      STREAM(actualCoM_MRE.z)
    STREAM_REGISTER_FINISH;
  };

  /** Constructor */
  FLIPMObservedState()
  {
    actualAcc = Vector3f::Zero();
    actualCoM_IMU = Point();
    actualCoM_MRE = Point();
  };

  /** Desctructor */
  ~FLIPMObservedState(){};

  Vector3f actualAcc;
  Point actualCoM_IMU;
  Point actualCoM_MRE;
};
