
#pragma once

#include "taskflow/taskflow.hpp"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/FieldColor.h"
#include "Representations/Perception/FieldColorMask.h"
#include "Tools/Module/Module.h"

#include <immintrin.h>
#include <vector>

struct Color
{
  uchar y;
  uchar cb;
  uchar cr;
};

struct HGrayscaleImage
{
  unsigned char* data;
  int width, height, widthStep;

  HGrayscaleImage(int width, int height)
  {
    data = (unsigned char*)_mm_malloc(width * height, 64);
    this->width = width;
    this->height = height;
    this->widthStep = width;
  }

  HGrayscaleImage(int width, int height, int widthStep, unsigned char* buffer)
  {
    data = buffer;
    this->width = width;
    this->height = height;
    this->widthStep = widthStep;
  }
};

STREAMABLE(PreprocessSettings,
  PreprocessSettings() = default;
  PreprocessSettings(int numRankFilters, int numDilations, int numErodes) : numRankFilters(numRankFilters) COMMA numDilations(numDilations) COMMA numErodes(numErodes) {}
  ,
  (int) numRankFilters,
  (int) numDilations,
  (int) numErodes
);

MODULE(FieldColorMaskProvider,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(FieldColors),
  REQUIRES(FieldColorsUpper),
  PROVIDES(FieldColorMask),
  PROVIDES(FieldColorMaskUpper),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    (int)(52) threshY,
    (int)(0) extraThreshCx,
    //(float) (60.f) lookingUpAngle,
    (PreprocessSettings) (PreprocessSettings(1, 2, 1)) lower,
    (PreprocessSettings) (PreprocessSettings(4, 0, 0)) upper
  )
);

class FieldColorMaskProvider : public FieldColorMaskProviderBase
{
private:
  FieldColorMask localFieldColorMask;
  FieldColorMaskUpper localFieldColorMaskUpper;

  void update(FieldColorMask& fieldColorMask) override;
  void update(FieldColorMaskUpper& fieldColorMaskUpper) override;
  void execute(tf::Subflow& subflow) override;
};
