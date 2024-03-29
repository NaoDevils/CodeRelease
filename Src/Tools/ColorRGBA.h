#pragma once

#include "Tools/Streams/InOut.h"

class ColorRGBA
{
public:
  unsigned char r = 0;
  unsigned char g = 0;
  unsigned char b = 0;
  unsigned char a = 255;

  static const ColorRGBA white;
  static const ColorRGBA black;
  static const ColorRGBA red;
  static const ColorRGBA green;
  static const ColorRGBA blue;
  static const ColorRGBA yellow;
  static const ColorRGBA cyan;
  static const ColorRGBA magenta;
  static const ColorRGBA orange;
  static const ColorRGBA violet;
  static const ColorRGBA gray;
  static const ColorRGBA purple;
  static const ColorRGBA brown;
  static const ColorRGBA darkgreen;

  ColorRGBA() = default;

  ColorRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) : r(r), g(g), b(b), a(a) {}

  ColorRGBA operator*(float scale) const
  {
    unsigned char r2 = static_cast<unsigned char>(scale * r);
    unsigned char g2 = static_cast<unsigned char>(scale * g);
    unsigned char b2 = static_cast<unsigned char>(scale * b);
    unsigned char a2 = static_cast<unsigned char>(scale * a);
    return ColorRGBA(r2, g2, b2, a2);
  }

  bool operator<(ColorRGBA second) const
  {
    if (r < second.r)
    {
      return true;
    }
    else
    {
      if (r == second.r)
      {
        if (g < second.g)
        {
          return true;
        }
        else
        {
          if (g == second.g)
          {
            if (b < second.b)
            {
              return true;
            }
            return false;
          }
          return false;
        }
      }
      return false;
    }
  }
};

In& operator>>(In& stream, ColorRGBA&);
Out& operator<<(Out& stream, const ColorRGBA&);
