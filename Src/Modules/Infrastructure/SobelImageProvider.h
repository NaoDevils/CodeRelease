#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/SobelImage.h"
#include "Tools/Debugging/DebugImages.h"
#include "Representations/Infrastructure/CameraResolution.h"

MODULE(SobelImageProvider,
{,
  REQUIRES(ImageUpper),
  REQUIRES(Image),
  REQUIRES(CameraResolution),
  PROVIDES(SobelImage),
  PROVIDES(SobelImageUpper),
  LOADS_PARAMETERS(
  {,
    ((Reduction) ReductionFactor) reduction,
    ((Reduction) ReductionFactor) reductionUpper,
    (bool) horizontal,
    (bool) vertical,
    (bool) magnitude,
    (bool) horizontalUpper,
    (bool) verticalUpper,
    (bool) magnitudeUpper,
    }),
});

class SobelImageProvider : public SobelImageProviderBase
{

public:
  DECLARE_DEBUG_IMAGE(SobelImageHorizontal);
  DECLARE_DEBUG_IMAGE(SobelImageVertical);
  DECLARE_DEBUG_IMAGE(SobelImageMagnitude);
  DECLARE_DEBUG_IMAGE(SobelImageUpperHorizontal);
  DECLARE_DEBUG_IMAGE(SobelImageUpperVertical);
  DECLARE_DEBUG_IMAGE(SobelImageUpperMagnitude);

  SobelImageProvider();

private:
  void update(SobelImage &sobelImage);
  void update(SobelImageUpper &sobelImageUpper);
};