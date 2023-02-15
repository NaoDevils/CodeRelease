/**
  * @file PNGLoggerSequence.h
  * This file declares a module that writes all camera images logged with the sequence provider to PNG files.
  * @author Arne Moos
  * @author Fabian Rensen
 **/

#pragma once
//#define STB_IMAGE_IMPLEMENTATION
//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "Tools/Module/Module.h"

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
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Perception/PNGImage.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Platform/File.h"
#include <string>
#include "Tools/ColorModelConversions.h"
#include <time.h>


MODULE(PNGLoggerSequence,
  REQUIRES(SequenceImage),
  REQUIRES(SequenceImageUpper),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraIntrinsics),
  REQUIRES(BehaviorData),
  PROVIDES(PNGImageDummy),

  LOADS_PARAMETERS(,
    (bool) enabled,                 // bool to determine if the PNG-Logger is enabled or not
    (bool) upper,                   // log the upper image (if one of the other log options is enabled)
    (bool) lower,                   // log the lower image (if one of the other log options is enabled)
    (bool) enableProtobuf,          // writes Protobuf informations if true
    (std::string) datasetName,      // datasetName for Protobuf
    (std::vector<std::string>) datasetNameBlacklist
  )
);


class PNGLoggerSequence : public PNGLoggerSequenceBase
{
public:
  PNGLoggerSequence();

private:
  std::string lastDatasetName;
  bool lastEnabledState;

  void update(PNGImageDummy& dummy);

  std::string logfile;
  std::string filepathLower;
  std::string filepathUpper;
  std::string getFilePath(bool upper);
  void create_required_dirs();
  void create_subdirs(const char* path);

  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;
  unsigned lastImageTimeStampWhenSequenceBegun;
  unsigned lastImageUpperTimeStampWhenSequenceBegun;

  void logImage(bool upper);

  std::string fillLeadingZeros(unsigned number);
  std::string theDate;
  std::string getDate();

  unsigned char targetLower[320 * 240 * 3];
  unsigned char targetUpper[640 * 480 * 3];
};
