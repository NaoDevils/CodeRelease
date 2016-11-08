#pragma once
#include "Tools/Streams/Streamable.h"

class WhistlePlay : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(whistleCausedPlay);
    STREAM_REGISTER_FINISH;
  }

public:
  bool whistleCausedPlay;
};
