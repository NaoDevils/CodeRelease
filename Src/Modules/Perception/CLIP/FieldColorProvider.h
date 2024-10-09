/**
* @file FieldColorProvider.h
* Declaration of class FieldColorProvider
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/DebugImages.h"
#include <algorithm>

MODULE(FieldColorProvider,
  REQUIRES(FallDownState),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  PROVIDES(FieldColors),
  PROVIDES(FieldColorsUpper),
  LOADS_PARAMETERS(,
    (int) maxPixelCountImageFull,
    (int) maxPixelCountImagePart,
    (bool) useAreaBasedFieldColor,
    (int) maxDiffOptCr,
    (int) maxDiffOptCb,
    (int) maxDiffOptY,
    (int) maxDiffCbCrRatio,
    (int)(30) minLineToFieldColorThresholdUpper,
    (int)(30) minLineToFieldColorThresholdLower
  )
);

class FieldColorProvider : public FieldColorProviderBase
{
public:
  /**
  * Default constructor.
  */
  FieldColorProvider();

  float histY[64];
  float histCr[64];
  float histCb[64];

  Image::Pixel samples[2000];
  int sampleNo;

  float maxY, maxCr, maxCb, oldMax;

  float minYDivFactor;

  FieldColors localFieldColorLower;
  FieldColorsUpper localFieldColorUpper;

  DECLARE_DEBUG_IMAGE(FieldColor);
  DECLARE_DEBUG_IMAGE(FieldColorUpper);

private:
  void update(FieldColors& theFieldColor);
  void update(FieldColorsUpper& theFieldColorUpper);

  void execute(const bool& upper);
  void buildSamples(const bool& upper, const Vector2i& lowerLeft, const Vector2i& upperRight, const int& sampleSize);
  void calcFieldColorFromSamples(const bool& upper, FieldColors::FieldColor& fieldColor);
  void smoothFieldColors(const bool& upper);

  void draw(const bool& upper);

  int fieldColorWeighted(const Image::Pixel& p, const int& optCr, const int& maxY);
};
