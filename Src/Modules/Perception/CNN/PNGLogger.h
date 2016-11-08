/**
  * @file PNGLogger.h
  * This file declares a module that writes all camera images to PNG files.
  * @author Fabian Rensen
 **/

#pragma once
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/ImageProcessing/stb_image.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Representations/Perception/CNN/PNGImage.h"
#include "Platform/File.h"
#include <string>
#include "Tools/ColorModelConversions.h"
#include <time.h>


MODULE(PNGLogger,
{,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  PROVIDES(PNGImageDummy),

  LOADS_PARAMETERS(
  {,
    (bool) enabled,
    (bool) streamAsRGB,
    (int) everyXFrame,
  }),
});


class PNGLogger : public PNGLoggerBase
{
public:
  PNGLogger();

private:
  void update(PNGImageDummy& dummy);

  std::string getFilePath();

  std::string filepath;

  void logImage(bool upper);

  std::string getDate();

  int imageCounter = 0;

  std::string theDate;

  unsigned char targetLower[320*240*3];
  unsigned char targetUpper[640*480*3];


};
