/**
 * @file Modules/Infrastructure/LowFrameRateImageProvider.h
 * This file implements a module that provides an image that is only rarely updated.
 * The image is intended for logging purposes.
 * @author Arne Böckmann
 */

#include "LowFrameRateImageProvider.h"

void LowFrameRateImageProvider::update(LowFrameRateImage& lowFrameRateImage)
{
  lowFrameRateImage.imageUpdated = false;

  /*if(perceptsOnly)
  {
    if(logGoalPercept && theGoalPercept.goalPosts.size() > 0)
    {
      logCurrentImage(lowFrameRateImage);
      storeNextImage = false;
    }
  }
  else*/ //let logCurrentImage decided whether it wants to log or not
  {
    logCurrentImage(lowFrameRateImage);
  }
}

void LowFrameRateImageProvider::update(LowFrameRateImageUpper& lowFrameRateImage)
{
  lowFrameRateImage.imageUpdated = false;

  if (storeNextImage)
  {
    logCurrentImage((LowFrameRateImage&)lowFrameRateImage);
  }
}

void LowFrameRateImageProvider::logCurrentImage(LowFrameRateImage& lowFrameRateImage)
{
  if(theFrameInfo.getTimeSince(lastUpdateTime) >= 1000 / frameRate)
  { // Generate new image
    lastUpdateTime = theFrameInfo.time;
    updateImage(lowFrameRateImage, false);
    storeNextImage = true; // Store next image as well to make sure to get both upper and lower cam images
  }
  else if(storeNextImage)
  {
    updateImage(lowFrameRateImage, true);
    lastUpdateTimeUpper = theFrameInfo.time;
    storeNextImage = false;
  }
}
void LowFrameRateImageProvider::updateImage(LowFrameRateImage& lfrImage, bool upper) const
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  lfrImage.image.setImage(const_cast<Image::Pixel*>(image[0]));
  lfrImage.image.setResolution(image.width, image.height, image.isFullSize);
  lfrImage.image.timeStamp = image.timeStamp;
  lfrImage.imageUpdated = true;
}

MAKE_MODULE(LowFrameRateImageProvider, cognitionInfrastructure)
