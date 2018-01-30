/**
 * @file Image.cpp
 *
 * Implementation of struct Image.
 */

#include <cstring>

#include "Image.h"
#include "Tools/ColorModelConversions.h"
#include "Platform/BHAssert.h"

const int Image::maxResolutionWidth;
const int Image::maxResolutionHeight;

Image::Image(bool initialize, int width, int height) :
  width(width),
  height(height),
  widthStep(width * 2)
{
  // allocate full size image and keep it that way indepentent of resolution
  image = new Pixel[maxResolutionWidth * maxResolutionHeight * 2];
  if(initialize)
    for(int y = 0; y < height; ++y)
      for(Pixel* p = (*this)[y], *pEnd = p + width; p < pEnd; ++p)
        p->color = 0x80008000;
}

Image::Image(const Image& other) :
  isReference(true)
{
  *this = other;
}

Image::~Image()
{
  if(!isReference)
    delete[] image;
}

Image& Image::operator=(const Image& other)
{
  height = other.height;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timeStamp;
  isFullSize = other.isFullSize;

  if(isReference)
  {
    // allocate full size image and keep it that way independent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }

  const int size = width * sizeof(Pixel)* (isFullSize ? 2 : 1);
  for(int y = 0; y < height; ++y)
    memcpy((*this)[y], other[y], size);

  return *this;
}

void Image::setImage(unsigned char* buffer)
{
  setImage(reinterpret_cast<Pixel*>(buffer));
}

bool Image::projectIntoImage(int &x, int &y, int sizeX, int sizeY) const
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

void Image::setImage(Pixel* buffer)
{
  if(!isReference)
  {
    delete[] image;
    isReference = true;
  }
  image = buffer;
}

void Image::setImageBySSECopy(const Image &other, bool halfResolution) {
  ASSERT(this->height == other.height);
  ASSERT(this->width == other.width);

  Pixel* row = other.image;

  for (int y = 0; y < height*2; ++y) {
    for (int x = 0; x < width; x+=4) {
      __m128i load;
      if (!halfResolution || y % 2 == 0){
        load = _mm_loadu_si128(reinterpret_cast<__m128i*>(row + x));
      } else {
        load = _mm_setzero_si128();
      }
      _mm_storeu_si128(reinterpret_cast<__m128i*>(image + x + width*y), load);
    }
    row += width;
  }

}


void Image::convertFromYCbCrToRGB(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToRGB(ycbcrImage[y][x].y,
                                            ycbcrImage[y][x].cb,
                                            ycbcrImage[y][x].cr,
                                            (*this)[y][x].r,
                                            (*this)[y][x].g,
                                            (*this)[y][x].b);
}

void Image::convertFromRGBToYCbCr(const Image& rgbImage)
{
  height = rgbImage.height;
  width = rgbImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromRGBToYCbCr(rgbImage[y][x].r,
                                            rgbImage[y][x].g,
                                            rgbImage[y][x].b,
                                            (*this)[y][x].y,
                                            (*this)[y][x].cb,
                                            (*this)[y][x].cr);
}

void Image::convertFromYCbCrToHSI(const Image& ycbcrImage)
{
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromYCbCrToHSI(ycbcrImage[y][x].y,
                                            ycbcrImage[y][x].cb,
                                            ycbcrImage[y][x].cr,
                                            (*this)[y][x].h,
                                            (*this)[y][x].s,
                                            (*this)[y][x].i);
}

void Image::convertFromHSIToYCbCr(const Image& hsiImage)
{
  height = hsiImage.height;
  width = hsiImage.width;
  for(int y = 0; y < height; ++y)
    for(int x = 0; x < width; ++x)
      ColorModelConversions::fromHSIToYCbCr(hsiImage[y][x].h,
                                            hsiImage[y][x].s,
                                            hsiImage[y][x].i,
                                            (*this)[y][x].y,
                                            (*this)[y][x].cb,
                                            (*this)[y][x].cr);
}

void Image::setResolution(int newWidth, int newHeight, bool fullSize)
{
  width = newWidth;
  height = newHeight;
  widthStep = width * 2;
  isFullSize = fullSize;
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

void Image::copyAndResizeArea(const int xPos, const int yPos, int sizeX, int sizeY, int sizeXNew, int sizeYNew, std::vector<unsigned char> &result) const
{
  //ASSERT(result.size() == sizeXNew*sizeYNew);
  unsigned char* image_ptr = (unsigned char*)image;
  const int rowStep = widthStep * 4;
  image_ptr += yPos*rowStep + xPos * 4;
  unsigned char* row_ptr = image_ptr;
  float xPrecise = 0.f;
  const float xStep = static_cast<float>(sizeX) / static_cast<float>(sizeXNew);
  float yPrecise = 0.f;
  const float yStep = static_cast<float>(sizeY) / static_cast<float>(sizeYNew);
  int pixelNo = 0;
  int lastX = 0;
  int lastY = 0;
  for (int y = 0; y < sizeYNew; y++)
  {
    for (int x = 0; x < sizeXNew; x++)
    {
      result[pixelNo] = *image_ptr;
      xPrecise += xStep;
      int xInc = static_cast<int>(xPrecise + 0.5) - lastX;
      lastX += xInc;
      image_ptr += xInc * 4;
      pixelNo++;
    }
    yPrecise += yStep;
    int yInc = static_cast<int>(yPrecise + 0.5) - lastY;
    lastY += yInc;
    row_ptr += yInc * rowStep;
    image_ptr = row_ptr;
  }
}

void Image::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);
  if(isFullSize)
    timeStamp |= 1 << 31;
  STREAM(timeStamp);
  isFullSize = (timeStamp & 1 << 31) != 0;
  timeStamp &= ~(1 << 31);

  const int size = width * sizeof(Pixel) * (isFullSize ? 2 : 1);

  if(out)
    for(int y = 0; y < height; ++y)
      out->write((*this)[y], size);
  else
  {
    widthStep = width * 2;
    for(int y = 0; y < height; ++y)
      in->read((*this)[y], size);
  }

  STREAM_REGISTER_FINISH;
}
