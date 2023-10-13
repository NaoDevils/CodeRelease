#include "ImagePatch.h"
#include "Representations/Infrastructure/Image.h"

const std::vector<unsigned char>& ImagePatch::getPatch() const
{
  if (patch.empty())
  {
    patch.resize(outputSize.prod() * (rgb ? 3 : 1));
    if (rgb)
      image->copyAndResizeArea<true>(inputPosition, inputSize, outputSize, patch.data());
    else
      image->copyAndResizeArea<false>(inputPosition, inputSize, outputSize, patch.data());
  }

  return patch;
}

void ImagePatch::setPatch(const Image& image, const Vector2i& inputPosition, const Vector2i& inputSize, const Vector2i& outputSize, bool rgb)
{
  this->inputPosition = inputPosition;
  this->inputSize = inputSize;
  this->outputSize = outputSize;
  this->rgb = rgb;
  this->image = &image;
  patch.clear();
}

void ImagePatch::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(rgb);
  if (out && image && patch.empty())
  {
    Streaming::registerDefaultElement(patch);
    Streaming::registerWithSpecification("patch", typeid(patch.data()));

    out->select("patch", -1);
    *out << static_cast<unsigned>(outputSize.prod() * (rgb ? 3 : 1));
    if (rgb)
      image->copyAndResizeArea<true>(inputPosition, inputSize, outputSize, out);
    else
      image->copyAndResizeArea<false>(inputPosition, inputSize, outputSize, out);
    out->deselect();
  }
  else
  {
    STREAM(patch);
  }
  STREAM_REGISTER_FINISH;
}
