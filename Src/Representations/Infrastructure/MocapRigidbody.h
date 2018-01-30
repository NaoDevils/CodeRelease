/**
* @class MocapRigidbody.h
* Contains the parameters of a mocap rigidbody
* @author <a href="mailto:janine.hemmers@tu-dortmund.de>Janine Hemmers</a>
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"

struct MocapRigidbody : public Streamable
{
  int id;
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;
  float meanError;
  bool trackingValid;
  int numberOfMarkers;
  Vector3f firstMarker;
  
  /** Constructor */
  MocapRigidbody():id(-1),trackingValid(false) {}
  MocapRigidbody(int id, float x, float y, float z, float qx, float qy, float qz, float qw): id(id),x(x),y(y),z(z),qx(qx),qy(qy),qz(qz),qw(qw),meanError(0),trackingValid(0) {}

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(id)
    STREAM(x)
    STREAM(y)
    STREAM(z)
    STREAM(qx)
    STREAM(qy)
    STREAM(qz)
    STREAM(qw)
    STREAM(meanError)
    STREAM(trackingValid)
    STREAM_REGISTER_FINISH;
  };

  /** Descructor */
  ~MocapRigidbody()
  {};

};