/**
* @file Modules/Infrastructure/SequenceImageProvider.h
* This file implements a module that provides a sequence of full Resolution Images.
* The image is intended as input for recurrent networks.
* @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a> 
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/BehaviorControl/BallSymbols.h"
MODULE(SequenceImageProvider,
{,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(BallSymbols),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  PROVIDES_WITHOUT_MODIFY(SequenceImage),
  PROVIDES_WITHOUT_MODIFY(SequenceImageUpper),
  LOADS_PARAMETERS(
  {,
    (unsigned) consecutiveFrames, /**< Number of consecutive Frames to be logged. */
    (unsigned) pausedFrames, /**< Number of Frames paused between logging */
  }),
});

class SequenceImageProvider : public SequenceImageProviderBase
{
public:
  SequenceImageProvider() : currentCounterOfConsecutiveFrames(0), currentCounterOfPausedFrames(pausedFrames), modifiedPausedFrames(pausedFrames), storeUpperImage(false) {}

  void update(SequenceImage& image);
  void update(SequenceImageUpper& image);

private:
  void logCurrentImage(SequenceImage& sequenceImage, bool upper);
  void updateImage(SequenceImage& lfrImage, bool upper) const;
  
  unsigned currentCounterOfConsecutiveFrames;
  unsigned currentCounterOfPausedFrames;
  unsigned modifiedPausedFrames;
  bool storeUpperImage;
};
