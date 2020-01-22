/**
 * @file Modules/Infrastructure/SequenceImageProvider.cpp
 * This file implements a module that provides a sequence of full Resolution Images.
 * The image is intended as input for recurrent networks.
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a> 
 */

#include "SequenceImageProvider.h"

void SequenceImageProvider::update(SequenceImage& lowFrameRateImage)
{
  lowFrameRateImage.noInSequence = 0;
  logCurrentImage(lowFrameRateImage, false);
}

void SequenceImageProvider::update(SequenceImageUpper& lowFrameRateImage)
{
  lowFrameRateImage.noInSequence = 0;
  logCurrentImage((SequenceImage&)lowFrameRateImage, true);
}

void SequenceImageProvider::logCurrentImage(SequenceImage& lowFrameRateImage, bool upper)
{
  if (theGameInfo.state == STATE_PLAYING && theCameraMatrix.isValid && theCameraMatrixUpper.isValid) {
    if (theBallSymbols.ballWasSeen 
      && currentCounterOfConsecutiveFrames == 0 
      && currentCounterOfPausedFrames > 60 
      && currentCounterOfPausedFrames < modifiedPausedFrames
      && modifiedPausedFrames < (3 * pausedFrames))
    { //start logging again when ball was seen
      modifiedPausedFrames += std::max(0, static_cast<int>(pausedFrames) - static_cast<int>(currentCounterOfPausedFrames)); // add the missing paused frames to the next waiting time
      currentCounterOfPausedFrames = modifiedPausedFrames;
    }

    if (storeUpperImage && upper)
    {
      updateImage(lowFrameRateImage, upper);
      storeUpperImage = false;
    }
    else if (!upper)
    {
      if (currentCounterOfPausedFrames >= modifiedPausedFrames)
      {
        currentCounterOfConsecutiveFrames++;
        updateImage(lowFrameRateImage, upper);
        storeUpperImage = true;
      }
      else
      {
        currentCounterOfPausedFrames++;
        if (currentCounterOfPausedFrames >= modifiedPausedFrames)
          modifiedPausedFrames = pausedFrames;
      }

      if (currentCounterOfConsecutiveFrames >= consecutiveFrames)
      {
        currentCounterOfPausedFrames = 0;
        currentCounterOfConsecutiveFrames = 0;
      }
    }
  }
  else {
    currentCounterOfPausedFrames++;
  }
}

void SequenceImageProvider::updateImage(SequenceImage& lfrImage, bool upper) const
{
  const Image& image = upper ? (Image&)theImageUpper : theImage;
  lfrImage.image.setImage(const_cast<Image::Pixel*>(image[0]));
  lfrImage.image.setResolution(image.width, image.height, image.isFullSize);
  lfrImage.image.timeStamp = image.timeStamp;
  lfrImage.noInSequence = currentCounterOfConsecutiveFrames;
}

MAKE_MODULE(SequenceImageProvider, cognitionInfrastructure)
