#pragma once

#include "Tools/Streams/Streamable.h"

struct Image;

struct ImagePatch : public Streamable
{
  const std::vector<unsigned char>& getPatch() const;
  void setPatch(const Image& image, const Vector2i& inputPosition, const Vector2i& inputSize, const Vector2i& outputSize, bool rgb = true);

  void serialize(In* in, Out* out);

  bool rgb = true;

protected:
  Vector2i inputPosition = Vector2i::Zero();
  Vector2i inputSize = Vector2i::Zero();
  Vector2i outputSize = Vector2i::Zero();

private:
  mutable std::vector<unsigned char> patch; // mutable needed in const getPatch()
  const Image* image = nullptr;
};
