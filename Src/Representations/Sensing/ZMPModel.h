/**
* @file ZMPModel.h
*
* Declaration of class ZMPModel
*
* @author <A href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*/

#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Debugging/Watch.h"
#endif
#include "Tools/Math/Eigen.h"

/**
 * @class ZMPModel
 *
 * Contains information about ZMP.
 */
class ZMPModel : public Streamable
{
  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN;
	  STREAM(zmp_acc);
    STREAM(ZMP_WCS);
    STREAM_REGISTER_FINISH;
  }

public:
  Vector3f zmp_acc; //ZMP
  Vector3f ZMP_WCS; // ZMP in Walking Engine World Coordinate System

  /** Constructor */
  ZMPModel() : zmp_acc(Vector3f::Zero()),ZMP_WCS(Vector3f::Zero()) {}
};
