/**
 * @file Modules/Infrastructure/LowFrameRateImageProvider.h
 * This file declares a module that provides an image that is only rarely updated.
 * The image is intended for logging purposes.
 * NDevils adjustment: first log lower image and next frame log upper image.
 * NDevils TODO: can we log both at once?
 * @author Arne Böckmann
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/LowFrameRateImage.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(LowFrameRateImageProvider,
{,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(FrameInfo),
  PROVIDES_WITHOUT_MODIFY(LowFrameRateImage),
  PROVIDES_WITHOUT_MODIFY(LowFrameRateImageUpper),
  LOADS_PARAMETERS(
  {,
    (float) frameRate, /**< Frames per second. */
    (bool) perceptsOnly, /**<If true only images containing percepts will be logged */
    (bool) logGoalPercept, /**< If true images containing goal percepts will be logged. Only if perceptsOnly is true as well */
  }),
});

class LowFrameRateImageProvider : public LowFrameRateImageProviderBase
{
public:
  LowFrameRateImageProvider() : lastUpdateTime(0), lastUpdateTimeUpper(0), storeNextImage(false) {}
  void update(LowFrameRateImage& image);
  void update(LowFrameRateImageUpper& image);

private:
  void logCurrentImage(LowFrameRateImage& lowFrameRateImage);
  void updateImage(LowFrameRateImage& lfrImage, bool upper) const;

  unsigned lastUpdateTime; /**< Time of last update. */
  unsigned lastUpdateTimeUpper; /**< Time of last update for upper image. */
  bool storeNextImage;
};
