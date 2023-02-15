/**
* @file JPEGImageProvider.h
* This file declares a module that provides JPEG images.
* @author Aaron Larisch
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/JPEGImage.h"

MODULE(JPEGImageProvider,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  PROVIDES_CONCURRENT(JPEGImage),
  PROVIDES_CONCURRENT(JPEGImageUpper),
  LOADS_PARAMETERS(,
    (int)(85) quality
  )
);

class JPEGImageProvider : public JPEGImageProviderBase
{
public:
private:
  void update(JPEGImage&);
  void update(JPEGImageUpper&);
};
