/**
 * @file Image.cpp
 *
 * Implementation of struct Image.
 */
#include "Tools/SIMD.h"
#include <cstring>
#include <numeric>

#include "Image.h"
#include "Tools/ColorModelConversions.h"
#include "Platform/BHAssert.h"

Image::Image(bool initialize, int width, int height) : width(width), height(height), widthStep(width * 2)
{
  // allocate full size image and keep it that way indepentent of resolution
  image = new Pixel[maxResolutionWidth * maxResolutionHeight * 2];
  if (initialize)
    for (int y = 0; y < height; ++y)
      for (Pixel *p = (*this)[y], *pEnd = p + width; p < pEnd; ++p)
        p->color = 0x80008000;
}

Image::Image(const Image& other) : isReference(true)
{
  *this = other;
}

Image::Image(Image&& other) noexcept : isReference(true)
{
  *this = std::move(other);
}

Image::~Image()
{
  if (!isReference)
    delete[] image;
}

Image& Image::operator=(const Image& other)
{
  height = other.height;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timeStamp;
  imageSource = other.imageSource;

  if (isReference)
  {
    // allocate full size image and keep it that way independent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }

  const int size = width * sizeof(Pixel);
  for (int y = 0; y < height; ++y)
    memcpy((*this)[y], other[y], size);

  return *this;
}

Image& Image::operator=(Image&& other) noexcept
{
  height = other.height;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timeStamp;
  imageSource = other.imageSource;

  if (!isReference)
    delete[] image;

  isReference = other.isReference;
  image = other.image;
  other.isReference = true;
  other.image = nullptr;

  return *this;
}

void Image::setImage(unsigned char* buffer)
{
  setImage(reinterpret_cast<Pixel*>(buffer));
}

bool Image::isOutOfBounds(int xStart, int xOffset, int yStart, int yOffset) const
{
  return xStart + xOffset <= 0 || xStart + xOffset >= width || yStart + yOffset <= 0 || yStart + yOffset >= height;
}

bool Image::projectIntoImage(int& x, int& y, int sizeX, int sizeY) const
{
  if (sizeX > width || sizeY > height)
    return false;
  if (x < 0)
    x = 0;
  if (x + sizeX >= width)
    x = width - sizeX;
  if (y < 0)
    y = 0;
  if (y + sizeY >= height)
    y = height - sizeY;
  return true;
}

bool Image::projectIntoImage(Vector2i& center, float radius) const
{
  int size = static_cast<int>(radius + 0.5f);
  if (size > width || size > height)
    return false;
  if (center.x() - size < 0)
    center.x() = size;
  if (center.x() + size >= width)
    center.x() = width - size;
  if (center.y() - size < 0)
    center.y() = size;
  if (center.y() + size >= height)
    center.y() = height - size;
  return true;
}

std::array<Vector2i, 3> Image::projectIntoImage(const Vector2i& patchCenter, const Vector2i& patchSize) const
{
  std::array<Vector2i, 3> ret;
  auto& [min, max, size] = ret;

  const auto clamp = [&](const Vector2i& v)
  {
    return Vector2i(std::clamp(v.x(), 0, width - 1), std::clamp(v.y(), 0, height - 1));
  };

  size = clamp(patchSize);
  max = clamp(patchCenter + size / 2);
  min = clamp(max - size);

  return ret;
}

bool Image::shouldBeProcessed() const
{
  return this->imageSource != ImageSource::thumbnail && this->imageSource != ImageSource::yoloInput;
};

void Image::setImage(Pixel* buffer)
{
  if (!isReference)
  {
    delete[] image;
    isReference = true;
  }
  image = buffer;
}

void Image::setImageBySSECopy(const Image& other, bool halfResolution)
{
  ASSERT(this->height == other.height);
  ASSERT(this->width == other.width);

  Pixel* row = other.image;

  for (int y = 0; y < height * 2; ++y)
  {
    for (int x = 0; x < width; x += 4)
    {
      __m128i load;
      if (!halfResolution || y % 2 == 0)
      {
        load = _mm_loadu_si128(reinterpret_cast<__m128i*>(row + x));
      }
      else
      {
        load = _mm_setzero_si128();
      }
      _mm_storeu_si128(reinterpret_cast<__m128i*>(image + x + width * y), load);
    }
    row += width;
  }
}

void Image::convertFromYCbCrToRGB(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToRGB(ycbcrImage[y][x].y, ycbcrImage[y][x].cb, ycbcrImage[y][x].cr, (*this)[y][x].r, (*this)[y][x].g, (*this)[y][x].b);
}

void Image::convertFromRGBToYCbCr(const Image& rgbImage)
{
  height = rgbImage.height;
  width = rgbImage.width;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      ColorModelConversions::fromRGBToYCbCr(rgbImage[y][x].r, rgbImage[y][x].g, rgbImage[y][x].b, (*this)[y][x].y, (*this)[y][x].cb, (*this)[y][x].cr);
}

void Image::convertFromYCbCrToHSI(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToHSI(ycbcrImage[y][x].y, ycbcrImage[y][x].cb, ycbcrImage[y][x].cr, (*this)[y][x].h, (*this)[y][x].s, (*this)[y][x].i);
}

void Image::convertFromHSIToYCbCr(const Image& hsiImage)
{
  height = hsiImage.height;
  width = hsiImage.width;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      ColorModelConversions::fromHSIToYCbCr(hsiImage[y][x].h, hsiImage[y][x].s, hsiImage[y][x].i, (*this)[y][x].y, (*this)[y][x].cb, (*this)[y][x].cr);
}

void Image::setResolution(int newWidth, int newHeight)
{
  width = newWidth;
  height = newHeight;
  widthStep = width * 2;
}

float Image::getColorDistance(const Image::Pixel& a, const Image::Pixel& b)
{
  int dy = int(a.y) - b.y;
  int dcb = int(a.cb) - b.cb;
  int dcr = int(a.cr) - b.cr;
  dy *= dy;
  dcb *= dcb;
  dcr *= dcr;
  return std::sqrt(float(dy + dcb + dcr));
}

std::vector<int> Image::copyAndResizeRGBFloatNoHorizon(int sizeXNew, int sizeYNew, const int horizon_y, float* result) const
{
  const int rowStep = widthStep * 4;
  unsigned char* image_ptr = (unsigned char*)image;
  image_ptr += horizon_y * rowStep;
  unsigned char* row_ptr = image_ptr;
  int pixelNo = 0;

  const float xStep = static_cast<float>(width) / static_cast<float>(sizeXNew);
  float xPrecise = 0.f;
  int lastX = 0;

  // precompute x offsets
  std::vector<int> xIncs(sizeXNew);
  for (int i = 0; i < sizeXNew; i++)
  {
    xPrecise += xStep;
    xIncs[i] = static_cast<int>(xPrecise + 0.5) - lastX;
    lastX += xIncs[i];
  }

  // precompute y offsets
  std::vector<int> yIdxs(sizeYNew);
  for (int i = 0; i < sizeYNew; i++)
  {
    yIdxs[i] = static_cast<int>(horizon_y + i * (height - horizon_y) / sizeYNew + 0.5);
  }

  std::vector<int> const yIdxsTemp = yIdxs;
  int sixth = static_cast<int>(sizeYNew / 6);
  for (int i = 0; i < sixth; i++)
  {
    int first = yIdxsTemp[i];
    int second = yIdxsTemp[i + 1];
    int new_value = static_cast<int>((first + second) / 2);
    yIdxs.erase(yIdxs.begin() + (sizeYNew - 1 - i));
    yIdxs.insert(yIdxs.begin() + (i + i + 1), new_value);
  }
  yIdxs.push_back(height);

  unsigned char r, g, b, y, u, v;
  for (int yIndex = 0; yIndex < sizeYNew; yIndex++)
  {
    for (int xIndex = 0; xIndex < sizeXNew; xIndex++)
    {
      y = image_ptr[0];
      u = image_ptr[1];
      v = image_ptr[3];
      ColorModelConversions::fromYCbCrToRGB(y, u, v, r, g, b);
      result[pixelNo++] = r / 255.f;
      result[pixelNo++] = g / 255.f;
      result[pixelNo++] = b / 255.f;

      int xInc = xIncs[xIndex];
      image_ptr += xInc * 4;
    }
    int yInc = yIdxs[yIndex + 1] - yIdxs[yIndex];
    row_ptr += yInc * rowStep;
    image_ptr = row_ptr;
  }

  return yIdxs;
}

std::vector<int> Image::getIndices(const int inputSize, const int outputSize, const int inputPos)
{
  std::vector<int> ret(outputSize);

  if (inputSize == outputSize)
  {
    std::iota(ret.begin(), ret.end(), inputPos);
  }
  else
  {
    for (int i = 0; i < outputSize; ++i)
      ret[i] = static_cast<int>(std::round(static_cast<float>(i) / outputSize * inputSize)) + inputPos;
  }
  return ret;
}

void Image::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);
  STREAM(timeStamp);
  timeStamp &= ~(1 << 31); // remove legacy isFullSize bit

  const int size = width * sizeof(Pixel);

  if (out)
    for (int y = 0; y < height; ++y)
      out->write((*this)[y], size);
  else
  {
    widthStep = width * 2;
    for (int y = 0; y < height; ++y)
      in->read((*this)[y], size);
  }

  STREAM_REGISTER_FINISH;
}
