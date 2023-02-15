#include "ColorRGBA.h"

const ColorRGBA ColorRGBA::cyan(0, 255, 255);
const ColorRGBA ColorRGBA::red(255, 0, 0);
const ColorRGBA ColorRGBA::yellow(255, 255, 0);
const ColorRGBA ColorRGBA::black(0, 0, 0);
const ColorRGBA ColorRGBA::white(255, 255, 255);
const ColorRGBA ColorRGBA::darkgreen(13, 156, 0);
const ColorRGBA ColorRGBA::orange(255, 130, 0);
const ColorRGBA ColorRGBA::purple(117, 0, 255);
const ColorRGBA ColorRGBA::brown(107, 48, 23);
const ColorRGBA ColorRGBA::gray(128, 128, 128);

const ColorRGBA ColorRGBA::blue(0, 0, 255);
const ColorRGBA ColorRGBA::magenta(255, 0, 255);
const ColorRGBA ColorRGBA::violet(183, 10, 210);
const ColorRGBA ColorRGBA::green(0, 255, 0);

In& operator>>(In& stream, ColorRGBA& color)
{
  stream >> color.r;
  stream >> color.g;
  stream >> color.b;
  stream >> color.a;
  return stream;
}

Out& operator<<(Out& stream, const ColorRGBA& color)
{
  stream << color.r;
  stream << color.g;
  stream << color.b;
  stream << color.a;
  return stream;
}
