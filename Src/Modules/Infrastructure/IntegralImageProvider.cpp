#include "IntegralImageProvider.h"

IntegralImageProvider::IntegralImageProvider()
{

}

void IntegralImageProvider::update(IntegralImage &integralImage)
{
  if (theFrameInfo.time != timeStampLastExecuted)
  {
    timeStampLastExecuted = theFrameInfo.time;
    execute();
  }
  integralImage = lowerImage;
}
void IntegralImageProvider::update(IntegralImageUpper &integralImageUpper)
{
  if (theFrameInfo.time != timeStampLastExecuted)
  {
    timeStampLastExecuted = theFrameInfo.time;
    execute();
  }
  integralImageUpper = upperImage;
}

void IntegralImageProvider::execute()
{
  if (!initialized)
  {
    lowerImage.init(widthLower, heightLower);
    upperImage.init(widthUpper, heightUpper);
    INIT_DEBUG_IMAGE_BLACK(IntegralImage, widthLower, heightLower);
    INIT_DEBUG_IMAGE_BLACK(IntegralImageUpper, widthUpper, heightUpper);
  }
  if (createLower)
  {
    if (fromGrayLower)
    {
      if (fromSobelLower)
        lowerImage.createIntegralImageDiffFromGraySobel(theSobelImage, theFieldColors.fieldColorArray[0].fieldColorOptY);
      else
        lowerImage.createIntegralImageDiffFromGray(theImage, theFieldColors.fieldColorArray[0].fieldColorOptY);
    }
    else
    {
      if (fromSobelLower)
        lowerImage.createIntegralImageSobel(theSobelImage);
      else
        lowerImage.createIntegralImage(theImage);
    }
  }
  if (createUpper)
  {
    if (fromGrayUpper)
    {
      if (fromSobelUpper)
        upperImage.createIntegralImageDiffFromGraySobel(theSobelImageUpper, theFieldColorsUpper.fieldColorArray[0].fieldColorOptY);
      else
        upperImage.createIntegralImageDiffFromGray(theImageUpper, theFieldColorsUpper.fieldColorArray[0].fieldColorOptY);
    }
    else
    {
      if (fromSobelUpper)
        upperImage.createIntegralImageSobel(theSobelImageUpper);
      else
        upperImage.createIntegralImage(theImageUpper);
    }
  }
  COMPLEX_IMAGE(IntegralImage)
  {
    int width = lowerImage.width;
    int height = lowerImage.height;
    int rowIndex = 0;
    int lastRowIndex = -width;
    for (int y = 1; y < height; y++)
    {
      rowIndex += width;
      lastRowIndex += width;
      for (int x = 1; x < width; x++)
      {
        int pixelValue = lowerImage.image[rowIndex + x]
          + lowerImage.image[lastRowIndex + x - 1]
          - lowerImage.image[rowIndex + x - 1]
          - lowerImage.image[lastRowIndex + x];
        DEBUG_IMAGE_SET_PIXEL_YUV(IntegralImage, x, y, static_cast<unsigned char>(pixelValue), 127, 127);
      }
    }
    SEND_DEBUG_IMAGE(IntegralImage);
  }
  COMPLEX_IMAGE(IntegralImageUpper)
  {
    int width = upperImage.width;
    int height = upperImage.height;
    int rowIndex = 0;
    int lastRowIndex = -width;
    for (int y = 1; y < height; y++)
    {
      rowIndex += width;
      lastRowIndex += width;
      for (int x = 1; x < width; x++)
      {
        int pixelValue = upperImage.image[rowIndex + x]
          + upperImage.image[lastRowIndex + x - 1]
          - upperImage.image[rowIndex + x - 1]
          - upperImage.image[lastRowIndex + x];
        DEBUG_IMAGE_SET_PIXEL_YUV(IntegralImageUpper, x, y, static_cast<unsigned char>(pixelValue), 127, 127);
      }
    }
    SEND_DEBUG_IMAGE(IntegralImageUpper);
  }
}

MAKE_MODULE(IntegralImageProvider, perception);