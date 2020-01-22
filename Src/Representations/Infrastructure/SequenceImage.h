/**
 * @file Image.h
 *
 * Declaration of struct Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Image.h"

struct SequenceImage : public Streamable
{
  Image image;
  unsigned noInSequence = 0;

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(noInSequence);
      if(noInSequence > 0)
      {
        STREAM(image);
      }
    STREAM_REGISTER_FINISH;
  }
};

struct SequenceImageUpper : public Streamable
{
  ImageUpper image;
  unsigned noInSequence = 0;

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(noInSequence);
      if (noInSequence > 0)
      {
        STREAM(image);
      }
    STREAM_REGISTER_FINISH;
  }
};