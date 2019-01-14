#include "IntegralImage.h"

void IntegralImage::createIntegralImage(const Image &source)
{
  const int factor = source.width / width;
  const int stepWidth = 4 * factor;
  const int rowStep = 4 * source.width * factor * 2 - 4 * source.width;
  //const int fieldColorY = theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
  unsigned char* image_ptr = (unsigned char*)source.image;
  unsigned row = 0;
  unsigned column = 0;
  int index = 0;
  int indexLastRow = (height - 2) * width;
  for (unsigned x = 0; x < width; x++)
    ySum[indexLastRow + x] = 0;
  unsigned lastYSum = 0;
  unsigned lastIISum = 0;
  for (; row < height; row++)
  {
    for (; column < width; column++)
    {
      /*int imageValue = *image_ptr;
      int diffFromGray = imageValue - theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
      int diffFromGrayAbs = std::abs(diffFromGray);
      imageValue = diffFromGrayAbs + (diffFromGrayAbs / 2) * -(sgn(diffFromGray) - 1);*/

      lastYSum = ySum[indexLastRow + column];
      ySum[index] = (lastYSum + *image_ptr);
      image[index] = lastIISum + ySum[index];
      lastIISum = image[index];
      index++;
      image_ptr += stepWidth;
    }
    lastIISum = 0;
    column = 0;
    indexLastRow = width * row;
    image_ptr += rowStep;
  }
}

void IntegralImage::createIntegralImageDiffFromGray(const Image &source, const unsigned grayValue)
{
  const int factor = source.width / width;
  const int stepWidth = 4 * factor;
  const int rowStep = 4 * source.width * factor * 2 - 4 * source.width;
  //const int fieldColorY = theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
  unsigned char* image_ptr = (unsigned char*)source.image;
  unsigned row = 0;
  unsigned column = 0;
  int index = 0;
  int indexLastRow = (height - 2) * width;
  for (unsigned x = 0; x < width; x++)
    ySum[indexLastRow + x] = 0;
  unsigned lastYSum = 0;
  unsigned lastIISum = 0;
  for (; row < height; row++)
  {
    for (; column < width; column++)
    {
      int imageValue = *image_ptr;
      int diffFromGray = imageValue - grayValue;
      int diffFromGrayAbs = std::abs(diffFromGray);
      imageValue = diffFromGrayAbs + (diffFromGrayAbs / 2) * -(sgn(diffFromGray) - 1);

      lastYSum = ySum[indexLastRow + column];
      ySum[index] = (lastYSum + imageValue);
      image[index] = lastIISum + ySum[index];
      lastIISum = image[index];
      index++;
      image_ptr += stepWidth;
    }
    lastIISum = 0;
    column = 0;
    indexLastRow = width * row;
    image_ptr += rowStep;
  }
}

void IntegralImage::createIntegralImageSobel(const SobelImage &source)
{
  const int factor = source.width / width;
  const int stepWidth = factor;
  const int rowStep = source.width * factor - source.width;
  //const int fieldColorY = theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
  unsigned const char* image_ptr = source.magnitude.data();
  unsigned row = 0;
  unsigned column = 0;
  int index = 0;
  int indexLastRow = (height - 2) * width;
  for (unsigned x = 0; x < width; x++)
    ySum[indexLastRow + x] = 0;
  unsigned lastYSum = 0;
  unsigned lastIISum = 0;
  for (; row < height; row++)
  {
    for (; column < width; column++)
    {
      /*int imageValue = *image_ptr;
      int diffFromGray = imageValue - theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
      int diffFromGrayAbs = std::abs(diffFromGray);
      imageValue = diffFromGrayAbs + (diffFromGrayAbs / 2) * -(sgn(diffFromGray) - 1);*/

      lastYSum = ySum[indexLastRow + column];
      ySum[index] = (lastYSum + *image_ptr);
      image[index] = lastIISum + ySum[index];
      lastIISum = image[index];
      index++;
      image_ptr += stepWidth;
    }
    lastIISum = 0;
    column = 0;
    indexLastRow = width * row;
    image_ptr += rowStep;
  }
}

void IntegralImage::createIntegralImageDiffFromGraySobel(const SobelImage &source, const unsigned grayValue)
{
  const int factor = source.width / width;
  const int stepWidth = factor;
  const int rowStep = source.width * factor - source.width;
  //const int fieldColorY = theFieldColorsUpper.fieldColorArray[0].fieldColorOptY;
  unsigned const char* image_ptr = source.magnitude.data();
  unsigned row = 0;
  unsigned column = 0;
  int index = 0;
  int indexLastRow = (height - 2) * width;
  for (unsigned x = 0; x < width; x++)
    ySum[indexLastRow + x] = 0;
  unsigned lastYSum = 0;
  unsigned lastIISum = 0;
  for (; row < height; row++)
  {
    for (; column < width; column++)
    {
      int imageValue = *image_ptr;
      int diffFromGray = imageValue - grayValue;
      int diffFromGrayAbs = std::abs(diffFromGray);
      imageValue = diffFromGrayAbs + (diffFromGrayAbs / 2) * -(sgn(diffFromGray) - 1);

      lastYSum = ySum[indexLastRow + column];
      ySum[index] = (lastYSum + imageValue);
      image[index] = lastIISum + ySum[index];
      lastIISum = image[index];
      index++;
      image_ptr += stepWidth;
    }
    lastIISum = 0;
    column = 0;
    indexLastRow = width * row;
    image_ptr += rowStep;
  }
}
