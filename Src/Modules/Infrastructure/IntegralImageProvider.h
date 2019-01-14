#pragma once

#include "Representations/Infrastructure/IntegralImage.h"
#include "Representations/Infrastructure/SobelImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/FieldColor.h"
#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugImages.h"

MODULE(IntegralImageProvider,
{ ,
  REQUIRES(ImageUpper),
  REQUIRES(Image),
  REQUIRES(SobelImageUpper),
  REQUIRES(SobelImage),
  REQUIRES(FrameInfo),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  PROVIDES(IntegralImage),
  PROVIDES(IntegralImageUpper),
  LOADS_PARAMETERS(
  { ,
    (unsigned) widthLower,
    (unsigned) heightLower,
    (unsigned) widthUpper,
    (unsigned) heightUpper,
    (bool) fromGrayLower,
    (bool) fromSobelLower,
    (bool) createLower,
    (bool) fromGrayUpper,
    (bool) fromSobelUpper,
    (bool) createUpper,
  }),
});

class IntegralImageProvider : public IntegralImageProviderBase
{

public:
  DECLARE_DEBUG_IMAGE(IntegralImage);
  DECLARE_DEBUG_IMAGE(IntegralImageUpper);

  IntegralImageProvider();

private:
  void update(IntegralImage &integralImage);
  void update(IntegralImageUpper &integralImageUpper);
  void execute();

  unsigned timeStampLastExecuted = 0;

  IntegralImage lowerImage;
  IntegralImageUpper upperImage;
  bool initialized = false;
};