/**
* @file InegralImage.h
*
* Declaration of struct IntegralImage.
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Image.h"
#include "SobelImage.h"
#include "Tools/Math/BHMath.h"

STREAMABLE(IntegralImage,
{
  IntegralImage(const int w = 320, const int h = 240)
  {
    init(w,h);
  }
  void init(const int w = 320, const int h = 240)
  {
    width = w; height = h;
    image.reserve(width * height);
    ySum.reserve(width * height);
    for (unsigned int x = 0; x < width; x++)
      for (unsigned int y = 0; y < height; y++)
      {
        ySum.push_back(0);
        image.push_back(0);
      }
  }
  int getPixelAt(const int x, const int y) const { return (int)image[y * width + x]; }
  void createIntegralImage(const Image &source);
  void createIntegralImageDiffFromGray(const Image &source, const unsigned grayValue);
  void createIntegralImageSobel(const SobelImage &source);
  void createIntegralImageDiffFromGraySobel(const SobelImage &source, const unsigned grayValue);
  ,
  (unsigned) width,
  (unsigned) height,
  (std::vector<unsigned>) image,
  (std::vector<unsigned>) ySum,
});

struct IntegralImageUpper : public IntegralImage
{
  IntegralImageUpper() : IntegralImage(640, 480) {}
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(IntegralImage);
    STREAM_REGISTER_FINISH;
  }
};