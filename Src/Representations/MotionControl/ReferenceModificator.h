/**
* @file ReferenceModificator 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
*/
#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Tools/DynamicRingBuffer.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif
#include "Tools/Math/Eigen.h"

/**
* @class ReferenceModificator
*
*/
struct ReferenceModificator : public Streamable
{

  struct Time
  {
    int startZMP,
        startFoot[2];
  };
  /** Constructor */
  ReferenceModificator() {};

  /** Destructor */
  ~ReferenceModificator(){};

  /** Modificator in WCS for x modification in RCS */
  Point x;

  /** Modificator in WCS for y modification in RCS */
  Point y;

  /** The error that can be completely handeled by sidesteps */
  Eigen::Matrix<Vector3f, 2, 1> handledErr;

  const Point & operator[] (int i) const
  {
    if (i == 0)
      return x;
    else
      return y;
  }

  Point & operator[] (int i)
  {
    if (i == 0)
      return x;
    else
      return y;
  }

  union {
    struct {
      Time x, y;
    } time;
    Time aTime[2];
  };

  int creationTime;

private:
  void serialize(In* in,Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(x.x);
    STREAM(x.y);
    STREAM(y.x);
    STREAM(y.y);
    STREAM_REGISTER_FINISH;
  };
};
