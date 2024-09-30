#pragma once

#include "Representations/Infrastructure/Image.h"

#include <algorithm>
#include <vector>

typedef unsigned char uchar;

STREAMABLE(FieldColorMask,
  inline FieldColorMask() : mask(Image::maxResolutionWidth * Image::maxResolutionHeight) {}
  inline const uchar* operator[](const int y) const { return mask.data() + y * width; }
  inline uchar* operator[](const int y) { return mask.data() + y * width; }
  inline uchar operator()(const int x, const int y) const { return (*this)[y][x]; }

  void draw() const;
  ,
  (std::vector<uchar>) mask,
  (int)(320) width,
  (int)(240) height
);

/**
* @brief Template-Spezialisierung von std::swap für FieldColorMask.
*/
template <> inline void std::swap(FieldColorMask& a, FieldColorMask& b) noexcept
{
  std::swap(a.mask, b.mask);
  std::swap(a.width, b.width);
  std::swap(a.height, b.height);
}

STREAMABLE_WITH_BASE(FieldColorMaskUpper, FieldColorMask,
  ,
);
