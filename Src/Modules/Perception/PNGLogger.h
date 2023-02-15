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

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
#include "Tools/ImageProcessing/stb_image.h"
#include "Tools/ImageProcessing/stb_image_write.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "Tools/Protobuf/ProtobufTools.h"
#include "Representations/Perception/PNGImage.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Platform/File.h"
#include <string>
#include "Tools/ColorModelConversions.h"
#include <time.h>


MODULE(PNGLogger,
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraIntrinsics),
  REQUIRES(BehaviorData),
  PROVIDES(PNGImageDummy),

  LOADS_PARAMETERS(,
    (bool) enabled,                 // bool to determine if the PNG-Logger is enabled or not
    (bool) logYUV,                  // log image/imageUpper as YUV
    (bool) streamAsRGB,             // if logYUV is enabled -> save image as RGB
    (bool) logY,                    // log image/imageUpper only as Y ("half" resolution)
    (bool) logYFull,                // log image/imageUpper only as Y in full resolution
    (bool) upper,                   // log the upper image (if one of the other log options is enabled)
    (bool) lower,                   // log the lower image (if one of the other log options is enabled)
    (int) everyXFrame,              // determine every X frame to be logged (usefull if enabled on the nao)
    (bool) enableProtobuf,          // writes Protobuf informations if true
    (std::string) datasetName      // datasetName for Protobuf
  )
);


class PNGLogger : public PNGLoggerBase
{
public:
  PNGLogger();

private:
  std::string lastDatasetName;
  bool lastEnabledState;

  void update(PNGImageDummy& dummy);

  std::string filepathLower;
  std::string filepathUpper;
  std::string getFilePath(bool upper);
  void create_required_dirs();
  void create_subdirs(const char* path);

  int imageCounter;
  int imageNumber;
  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;

  void logImage(bool upper);
  void logImageYFull(bool upper);
  void logImageY(bool upper);

  std::string fillLeadingZeros(unsigned number);
  std::string theDate;
  std::string getDate();

  unsigned char targetLower[320 * 240 * 3];
  unsigned char targetUpper[640 * 480 * 3];

  unsigned char targetLowerFull[640 * 480 * 3];
  unsigned char targetUpperFull[1280 * 960 * 3];
};
