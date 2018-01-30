/**
* @class MocapMarkerSet.h
* Contains the parameters of a mocap marker set
* @author <a href="mailto:janine.hemmers@tu-dortmund.de>Janine Hemmers</a>
*/

#pragma once
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Eigen.h"

struct MocapMarkerSet : public Streamable
{
  std::string name;
  std::vector<Vector3f> marker;

  MocapMarkerSet(){}
  MocapMarkerSet(std::string theName) :name(theName) {}

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(name)
    STREAM(marker)
    STREAM_REGISTER_FINISH;
  };

  /** Descructor */
  ~MocapMarkerSet()
  {};
};