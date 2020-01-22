/**
* @file FallDownAngleReduction
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
*/
#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

#include "Tools/Math/Eigen.h"

/**
 * @class FallDownAngleReduction
 *
 */
class FallDownAngleReduction : public Streamable
{

public:

  /** Constructor */
  FallDownAngleReduction() {};

  /** Destructor */
  ~FallDownAngleReduction() {};

  Vector2f reductionFactor = Vector2f(1.f, 1.f);

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(reductionFactor);
    STREAM_REGISTER_FINISH;
  };
};
