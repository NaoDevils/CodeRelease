#include "Compressed.h"

#include <cmath>

void Vector2fCompressed::serialize(In* in, Out* out)
{
  if (out && out->isCompressed())
  {
    ASSERT(this->allFinite());
    const short x = static_cast<short>(std::round(std::clamp(this->x() / precision, min, max)));
    const short y = static_cast<short>(std::round(std::clamp(this->y() / precision, min, max)));

    const unsigned char lx = static_cast<unsigned char>(x & 0x00FF);
    const unsigned char ly = static_cast<unsigned char>(y & 0x00FF);
    const unsigned char uxy = static_cast<unsigned char>((x & 0x0F00) >> 4 | (y & 0x0F00) >> 8);

    *out << lx << ly << uxy;
    return;
  }
  else if (in && in->isCompressed())
  {
    unsigned char lx, ly, uxy;
    *in >> lx >> ly >> uxy;

    const int x = static_cast<char>(uxy & 0xF0) << 4 | lx;
    const int y = static_cast<char>((uxy & 0x0F) << 4) << 4 | ly;
    this->x() = static_cast<float>(x) * precision;
    this->y() = static_cast<float>(y) * precision;
    return;
  }

  STREAM_REGISTER_BEGIN
  Streaming::registerBase();
  if (in)
    *in >> *static_cast<Vector2f*>(this);
  else if (out)
    *out << *static_cast<Vector2f*>(this);
  STREAM_REGISTER_FINISH
}

void AngleCompressed::serialize(In* in, Out* out)
{
  if (in && in->isCompressed())
  {
    char a;
    *in >> a;
    *this = static_cast<Angle>(static_cast<float>(a) / std::numeric_limits<char>::max() * pi);
    return;
  }
  else if (out && out->isCompressed())
  {
    ASSERT(std::isfinite(*this));
    *out << static_cast<char>(std::round(angle.normalize() / pi * std::numeric_limits<char>::max()));
    return;
  }

  // This is not very nice, we should add this to basicTypeSpecification (similar to Angle class)
  STREAM_REGISTER_BEGIN
  STREAM(angle);
  STREAM_REGISTER_FINISH
}

void Pose2fCompressed::serialize(In* in, Out* out)
{
  AngleCompressed rotation = this->rotation;
  Vector2fCompressed translation = this->translation;
  STREAM_REGISTER_BEGIN
  STREAM(rotation)
  STREAM(translation)
  STREAM_REGISTER_FINISH
  this->rotation = rotation;
  this->translation = translation;
}

void ValidityCompressed::serialize(In* in, Out* out)
{
  if (in && in->isCompressed())
  {
    uint8_t a;
    *in >> a;
    *this = static_cast<float>(a) / std::numeric_limits<uint8_t>::max();
    return;
  }
  else if (out && out->isCompressed())
  {
    ASSERT(0.f <= validity && validity <= 1.f);
    *out << static_cast<uint8_t>(std::roundf(validity * std::numeric_limits<uint8_t>::max()));
    return;
  }

  // This is not very nice, we should add this to basicTypeSpecification (similar to Angle class)
  STREAM_REGISTER_BEGIN
  STREAM(validity);
  STREAM_REGISTER_FINISH
}
