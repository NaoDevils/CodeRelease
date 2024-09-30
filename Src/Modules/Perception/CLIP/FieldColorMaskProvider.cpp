#include "FieldColorMaskProvider.h"
#include "Tools/SortingNetworks.h"

#include "taskflow/algorithm/for_each.hpp"

#include <cmath>

void FieldColorMaskProvider::update(FieldColorMask& fieldColorMask)
{
  localFieldColorMask.draw();
  std::swap(fieldColorMask, localFieldColorMask);
}

void FieldColorMaskProvider::update(FieldColorMaskUpper& fieldColorMaskUpper)
{
  localFieldColorMaskUpper.draw();
  std::swap(fieldColorMaskUpper, localFieldColorMaskUpper);
}

static void dilateSSE(HGrayscaleImage& inputImage, HGrayscaleImage& outputImage)
{
  unsigned char* img = inputImage.data;
  int width = inputImage.width;
  int height = inputImage.height;
  int widthStep = inputImage.widthStep;
  unsigned char* dst = outputImage.data;
  outputImage.width = width;
  outputImage.height = height;
  outputImage.widthStep = width;

  for (int y = 1; y < height - 1; y++)
  {
    for (int x = 1; x < width - 1; x += 16)
    {
      __m128i reg = _mm_loadu_si128((__m128i*)(img + x + y * widthStep));
      __m128i topReg = _mm_loadu_si128((__m128i*)(img + x + (y - 1) * widthStep));
      __m128i botReg = _mm_loadu_si128((__m128i*)(img + x + (y + 1) * widthStep));
      __m128i leftReg = _mm_loadu_si128((__m128i*)(img + (x - 1) + y * widthStep));
      __m128i rightReg = _mm_loadu_si128((__m128i*)(img + (x + 1) + y * widthStep));

      __m128i imgReg = _mm_min_epu8(_mm_min_epu8(topReg, botReg), _mm_min_epu8(leftReg, rightReg));
      _mm_storeu_si128((__m128i*)(dst + x + y * width), _mm_min_epu8(reg, imgReg));
    }
  }

  memcpy(dst, dst + width, width);
  memcpy(dst + (height - 1) * width, img + (height - 2) * widthStep, width);
}

static void erodeSSE(HGrayscaleImage& inputImage, HGrayscaleImage& outputImage)
{
  unsigned char* img = inputImage.data;
  int width = inputImage.width;
  int height = inputImage.height;
  int widthStep = inputImage.widthStep;
  unsigned char* dst = outputImage.data;
  outputImage.width = width;
  outputImage.height = height;
  outputImage.widthStep = width;

  for (int y = 1; y < height - 1; y++)
  {
    for (int x = 1; x < width - 1; x += 16)
    {
      __m128i reg = _mm_loadu_si128((__m128i*)(img + x + y * widthStep));
      __m128i topReg = _mm_loadu_si128((__m128i*)(img + x + (y - 1) * widthStep));
      __m128i botReg = _mm_loadu_si128((__m128i*)(img + x + (y + 1) * widthStep));
      __m128i leftReg = _mm_loadu_si128((__m128i*)(img + (x - 1) + y * widthStep));
      __m128i rightReg = _mm_loadu_si128((__m128i*)(img + (x + 1) + y * widthStep));

      __m128i imgReg = _mm_max_epu8(_mm_max_epu8(topReg, botReg), _mm_max_epu8(leftReg, rightReg));
      _mm_storeu_si128((__m128i*)(dst + x + y * width), _mm_max_epu8(reg, imgReg));
    }
  }

  memcpy(dst, dst + width, width);
  memcpy(dst + (height - 1) * width, img + (height - 2) * widthStep, width);
}

static inline __m128i _mm_cmpge_epu8(__m128i a, __m128i b)
{
  return _mm_cmpeq_epi8(a, _mm_max_epu8(a, b));
}

static void segmentImageSSEInplaceUnrolled(unsigned char* mask, unsigned char* imgptr, int width, int height, const Color& bgColor, const Color& thresh, int stepWidth)
{
  const __m128i bgReg = _mm_setr_epi8(bgColor.y,
      bgColor.cb,
      bgColor.y,
      bgColor.cr,
      bgColor.y,
      bgColor.cb,
      bgColor.y,
      bgColor.cr,
      bgColor.y,
      bgColor.cb,
      bgColor.y,
      bgColor.cr,
      bgColor.y,
      bgColor.cb,
      bgColor.y,
      bgColor.cr);

  const __m128i threshReg = _mm_setr_epi8(
      thresh.y, thresh.cb, thresh.y, thresh.cr, thresh.y, thresh.cb, thresh.y, thresh.cr, thresh.y, thresh.cb, thresh.y, thresh.cr, thresh.y, thresh.cb, thresh.y, thresh.cr);

  const __m128i shuffleMask1 = _mm_setr_epi8(0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  const __m128i shuffleMask2 = _mm_setr_epi8(-1, -1, -1, -1, 0, 4, 8, 12, -1, -1, -1, -1, -1, -1, -1, -1);
  const __m128i shuffleMask3 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1, 0, 4, 8, 12, -1, -1, -1, -1);
  const __m128i shuffleMask4 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 4, 8, 12);
  const __m128i ones = _mm_set1_epi32(-1);

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 64)
    {
      __m128i imgReg = _mm_loadu_si128((__m128i*)(imgptr + x));
      __m128i diff = _mm_or_si128(_mm_subs_epu8(bgReg, imgReg), _mm_subs_epu8(imgReg, bgReg));
      //passedThresh contains if the indiviual bytes passed the threshold:
      __m128i passedThresh = _mm_cmpge_epu8(threshReg, diff);
      //*Magic* : by interpreting four bytes as an int and comparing it to ones
      // we can check if all bytes: U Y V Y of a pixel passed the threshhold
      __m128i passedAllThresh = _mm_cmpeq_epi32(passedThresh, ones);
      //Now we just convert the ints to bytes/chars using a shuffle and we are done
      __m128i shuffled = _mm_shuffle_epi8(passedAllThresh, shuffleMask1);

      //repeat 3 times to reduce write instructions
      imgReg = _mm_loadu_si128((__m128i*)(imgptr + x + 16));
      diff = _mm_or_si128(_mm_subs_epu8(bgReg, imgReg), _mm_subs_epu8(imgReg, bgReg));
      passedThresh = _mm_cmpge_epu8(threshReg, diff);
      passedAllThresh = _mm_cmpeq_epi32(passedThresh, ones);
      __m128i shuffled2 = _mm_shuffle_epi8(passedAllThresh, shuffleMask2);
      shuffled = _mm_add_epi8(shuffled, shuffled2);

      imgReg = _mm_loadu_si128((__m128i*)(imgptr + x + 32));
      diff = _mm_or_si128(_mm_subs_epu8(bgReg, imgReg), _mm_subs_epu8(imgReg, bgReg));
      passedThresh = _mm_cmpge_epu8(threshReg, diff);
      passedAllThresh = _mm_cmpeq_epi32(passedThresh, ones);
      shuffled2 = _mm_shuffle_epi8(passedAllThresh, shuffleMask3);
      shuffled = _mm_add_epi8(shuffled, shuffled2);

      imgReg = _mm_loadu_si128((__m128i*)(imgptr + x + 48));
      diff = _mm_or_si128(_mm_subs_epu8(bgReg, imgReg), _mm_subs_epu8(imgReg, bgReg));
      passedThresh = _mm_cmpge_epu8(threshReg, diff);
      passedAllThresh = _mm_cmpeq_epi32(passedThresh, ones);
      shuffled2 = _mm_shuffle_epi8(passedAllThresh, shuffleMask4);
      shuffled = _mm_add_epi8(shuffled, shuffled2);

      // invert the mask
      shuffled = _mm_xor_si128(shuffled, ones);

      _mm_storeu_si128((__m128i*)(mask), shuffled);
      mask += 16;
    }
    imgptr += stepWidth;
  }
}

static void applyRankFilter(unsigned char* in, unsigned char* out, int width, int height)
{
  for (int y = 1; y < height - 1; y++)
  {
    for (int x = 1; x < width - 1; x += 16)
    {
      __m128i center = _mm_loadu_si128((__m128i*)(in + x + y * width));
      __m128i top = _mm_loadu_si128((__m128i*)(in + x + (y - 1) * width));
      __m128i bottom = _mm_loadu_si128((__m128i*)(in + x + (y + 1) * width));
      __m128i left = _mm_loadu_si128((__m128i*)(in + (x - 1) + y * width));
      __m128i right = _mm_loadu_si128((__m128i*)(in + (x + 1) + y * width));

      __m128i res = rank2n5(center, top, bottom, left, right);
      _mm_storeu_si128((__m128i*)(out + x + y * width), res);
    }
  }
}

void FieldColorMaskProvider::execute(tf::Subflow& subflow)
{
  // Die Orientierung ist die dritte Spalte der Kameramatrix
  /*Vector3f lowerOrientation = theCameraMatrix.inverse().rotation * Vector3f(0.f, 0.f, 1.f);
  Vector3f upperOrientation = theCameraMatrixUpper.inverse().rotation * Vector3f(0.f, 0.f, 1.f);

  float lowerAngleToGround = 90.f - std::acos(lowerOrientation.z()) * 180.f / (float)M_PI;
  float upperAngleToGround = 90.f - std::acos(upperOrientation.z()) * 180.f / (float)M_PI;*/

  int numLowerRankFilters, numLowerDilations, numLowerErodes, numUpperRankFilters, numUpperDilations, numUpperErodes;
  /*if (lowerAngleToGround < lookingUpAngle)
  {
    numLowerRankFilters = lookingDown.numRankFilters;
    numLowerDilations = lookingDown.numDilations;
    numLowerErodes = lookingDown.numErodes;
  }
  else
  {
    numLowerRankFilters = lookingUp.numRankFilters;
    numLowerDilations = lookingUp.numDilations;
    numLowerErodes = lookingUp.numErodes;
  }

  if (upperAngleToGround < lookingUpAngle)
  {
    numUpperRankFilters = lookingDown.numRankFilters;
    numUpperDilations = lookingDown.numDilations;
    numUpperErodes = lookingDown.numErodes;
  }
  else
  {
    numUpperRankFilters = lookingUp.numRankFilters;
    numUpperDilations = lookingUp.numDilations;
    numUpperErodes = lookingUp.numErodes;
  }*/

  numLowerRankFilters = lower.numRankFilters;
  numLowerDilations = lower.numDilations;
  numLowerErodes = lower.numErodes;

  numUpperRankFilters = upper.numRankFilters;
  numUpperDilations = upper.numDilations;
  numUpperErodes = upper.numErodes;

  const Color bgColorLower = {
      (uchar)theFieldColors.fieldColorArray[0].fieldColorOptY, (uchar)theFieldColors.fieldColorArray[0].fieldColorOptCb, (uchar)theFieldColors.fieldColorArray[0].fieldColorOptCr};

  const Color thresholdLower = {
      (uchar)threshY, (uchar)(theFieldColors.fieldColorArray[0].fieldColorMaxDistCb + extraThreshCx), (uchar)(theFieldColors.fieldColorArray[0].fieldColorMaxDistCr + extraThreshCx)};

  const Color bgColorUpper = {
      (uchar)theFieldColorsUpper.fieldColorArray[0].fieldColorOptY, (uchar)theFieldColorsUpper.fieldColorArray[0].fieldColorOptCb, (uchar)theFieldColorsUpper.fieldColorArray[0].fieldColorOptCr};

  const Color thresholdUpper = {
      (uchar)threshY, (uchar)(theFieldColorsUpper.fieldColorArray[0].fieldColorMaxDistCb + extraThreshCx), (uchar)(theFieldColorsUpper.fieldColorArray[0].fieldColorMaxDistCr + extraThreshCx)};

  localFieldColorMask.width = theImage.width;
  localFieldColorMask.height = theImage.height;
  localFieldColorMaskUpper.width = theImageUpper.width;
  localFieldColorMaskUpper.height = theImageUpper.height;

  unsigned char* imagePtr = (unsigned char*)theImage.image;
  unsigned char* maskPtr = lowerMaskFrontBuffer.data();
  int height = theImage.height / 4;
  int width = theImage.width * 4;
  int maskStep = height * theImage.width;
  int widthStep = theImage.widthStep * 4;
  int imageStep = height * widthStep;
  tf::Task segLowerTf =
      subflow
          .for_each_index(0,
              4,
              1,
              [imagePtr, maskPtr, width, height, bgColorLower, thresholdLower, widthStep, maskStep, imageStep](int i)
              {
                segmentImageSSEInplaceUnrolled(maskPtr + i * maskStep, imagePtr + i * imageStep, width, height, bgColorLower, thresholdLower, widthStep);
              })
          .name("Segmentation Lower [FieldColorMaskProvider]");

  imagePtr = (unsigned char*)theImageUpper.image;
  maskPtr = upperMaskFrontBuffer.data();
  height = theImageUpper.height / 4;
  width = theImageUpper.width * 4;
  maskStep = height * theImageUpper.width;
  widthStep = theImageUpper.widthStep * 4;
  imageStep = height * widthStep;
  tf::Task segUpperTf =
      subflow
          .for_each_index(0,
              4,
              1,
              [imagePtr, maskPtr, width, height, bgColorUpper, thresholdUpper, widthStep, maskStep, imageStep](int i)
              {
                segmentImageSSEInplaceUnrolled(maskPtr + i * maskStep, imagePtr + i * imageStep, width, height, bgColorUpper, thresholdUpper, widthStep);
              })
          .name("Segmentation Upper [FieldColorMaskProvider]");

  tf::Task openingLowerTf =
      subflow
          .emplace(
              [this, numLowerRankFilters, numLowerDilations, numLowerErodes]()
              {
                for (int i = 0; i < numLowerRankFilters; i++)
                {
                  applyRankFilter(lowerMaskFrontBuffer.data(), lowerMaskBackBuffer.data(), theImage.width, theImage.height);
                  std::swap(lowerMaskFrontBuffer, lowerMaskBackBuffer);
                }

                for (int i = 0; i < numLowerDilations; i++)
                {
                  HGrayscaleImage input(theImage.width, theImage.height, theImage.width, lowerMaskFrontBuffer.data());
                  HGrayscaleImage output(theImage.width, theImage.height, theImage.width, lowerMaskBackBuffer.data());
                  dilateSSE(input, output);
                  std::swap(lowerMaskFrontBuffer, lowerMaskBackBuffer);
                }

                for (int i = 0; i < numLowerErodes; i++)
                {
                  HGrayscaleImage input(theImage.width, theImage.height, theImage.width, lowerMaskFrontBuffer.data());
                  HGrayscaleImage output(theImage.width, theImage.height, theImage.width, lowerMaskBackBuffer.data());
                  erodeSSE(input, output);
                  std::swap(lowerMaskFrontBuffer, lowerMaskBackBuffer);
                }

                std::swap(localFieldColorMask.mask, lowerMaskFrontBuffer);
              })
          .name("Erosion/Dilation Lower [FieldColorMaskProvider]");

  openingLowerTf.succeed(segLowerTf);

  tf::Task openingUpperTf =
      subflow
          .emplace(
              [this, numUpperRankFilters, numUpperDilations, numUpperErodes]()
              {
                for (int i = 0; i < numUpperRankFilters; i++)
                {
                  applyRankFilter(upperMaskFrontBuffer.data(), upperMaskBackBuffer.data(), theImageUpper.width, theImageUpper.height);
                  std::swap(upperMaskFrontBuffer, upperMaskBackBuffer);
                }

                for (int i = 0; i < numUpperDilations; i++)
                {
                  HGrayscaleImage input(theImageUpper.width, theImageUpper.height, theImageUpper.width, upperMaskFrontBuffer.data());
                  HGrayscaleImage output(theImageUpper.width, theImageUpper.height, theImageUpper.width, upperMaskBackBuffer.data());
                  dilateSSE(input, output);
                  std::swap(upperMaskFrontBuffer, upperMaskBackBuffer);
                }

                for (int i = 0; i < numUpperErodes; i++)
                {
                  HGrayscaleImage input(theImageUpper.width, theImageUpper.height, theImageUpper.width, upperMaskFrontBuffer.data());
                  HGrayscaleImage output(theImageUpper.width, theImageUpper.height, theImageUpper.width, upperMaskBackBuffer.data());
                  erodeSSE(input, output);
                  std::swap(upperMaskFrontBuffer, upperMaskBackBuffer);
                }

                std::swap(localFieldColorMaskUpper.mask, upperMaskFrontBuffer);
              })
          .name("Erosion/Dilation Upper [FieldColorMaskProvider]");

  openingUpperTf.succeed(segUpperTf);
}

MAKE_MODULE(FieldColorMaskProvider, perception);
