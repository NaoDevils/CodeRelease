/**
* @file SobelImage.h
*
* Declaration of struct SobelImage
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/ImageProcessing/Vector2D.h"

#define IMAGE_WIDTH_FULL 1280
#define IMAGE_HEIGHT_FULL 960
#define IMAGE_WIDTH_HALF 640
#define IMAGE_HEIGHT_HALF 480

STREAMABLE(SobelImage,
{
  SobelImage();
  SobelImage(int width, int height);
  unsigned char getHorizontal(const int y, const int x);
  const unsigned char getHorizontal(const int y, const int x) const;
  unsigned char getVertical(const int y, const int x);
  const unsigned char getVertical(const int y, const int x) const;
  unsigned char getMagnitude(const int y, const int x);
  const unsigned char getMagnitude(const int y, const int x) const;
  ,
  (int) width,
  (int) height,
  (std::vector<unsigned char>) gx,
  (std::vector<unsigned char>) gy,
  (std::vector<unsigned char>) magnitude,
});

inline SobelImage::SobelImage() : SobelImage(IMAGE_WIDTH_FULL, IMAGE_HEIGHT_FULL)
{
}

inline SobelImage::SobelImage(int width, int height) : width(width), height(height), gx(width * height, 0),
                                                       gy(width * height, 0), magnitude(width * height, 0)
{
}

inline unsigned char SobelImage::getHorizontal(const int y, const int x)
{
  return gx.at(y * width + x);
}

inline const unsigned char SobelImage::getHorizontal(const int y, const int x) const
{
  return gx.at(y * width + x);
}

inline unsigned char SobelImage::getVertical(const int y, const int x)
{
  return gy.at(y * width + x);
}

inline const unsigned char SobelImage::getVertical(const int y, const int x) const
{
  return gy.at(y * width + x);
}

inline unsigned char SobelImage::getMagnitude(const int y, const int x)
{
  return magnitude.at(y * width + x);
}

inline const unsigned char SobelImage::getMagnitude(const int y, const int x) const
{
  return magnitude.at(y * width + x);
}



struct SobelImageUpper : public SobelImage
{
  SobelImageUpper() : SobelImage(IMAGE_WIDTH_FULL, IMAGE_HEIGHT_FULL)
  {

  }
};

namespace SobelDir
{
  ENUM(SobelDirection,
  {,
    horizontal,
    vertical,
    bidirectional,
  });
}

namespace Reduction
{
  ENUM(ReductionFactor,
  {,
    full,
    half,
    quarter,
  });
}