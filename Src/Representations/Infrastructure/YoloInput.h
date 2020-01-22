/**
 * @file YoloInput.h
 *
 * Declaration of struct YoloInput
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Image.h"

struct YoloInput : public Streamable
{
  std::vector<float> image;
  int width, height, channel;
  unsigned timeStamp;
  bool imageUpdated = false; /**< True if image was updated this frame */

public:
  void toImage(Image& dest) const;

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(imageUpdated);
      if(imageUpdated)
      {
        STREAM(image);
        STREAM(width);
        STREAM(height); 
        STREAM(channel);
        STREAM(timeStamp);
      }
    STREAM_REGISTER_FINISH;
  }
};

struct YoloInputUpper : public YoloInput{};
