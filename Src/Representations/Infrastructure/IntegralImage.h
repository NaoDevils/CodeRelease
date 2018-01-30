/**
* @file InegralImage.h
*
* Declaration of struct IntegralImage
* Only grayscale and 160x120!
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/Image.h"

STREAMABLE(IntegralImage,
{
  IntegralImage()
  {
    lowerImage.reserve(160 * 120);
    upperImage.reserve(160 * 120);
    ySum.reserve(160 * 120);
    for (unsigned int x = 0; x < 160; x++)
      for (unsigned int y = 0; y < 120; y++)
      {
        ySum.push_back(0);
        lowerImage.push_back(0);
        upperImage.push_back(0);
      }
  }
  unsigned getPixelLower(const int x, const int y) { return lowerImage[y * 160 + x]; }
  unsigned getPixelUpper(const int x, const int y) { return upperImage[y * 160 + x]; }
  
  ,
  (std::vector<unsigned>) lowerImage,
  (std::vector<unsigned>) upperImage,
  (std::vector<unsigned>) ySum,
});