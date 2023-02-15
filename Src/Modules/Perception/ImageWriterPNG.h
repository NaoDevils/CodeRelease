/**
  * @file ImageWriterPNG.h
  * This file declares a module that writes all logged camera images or patches to PNG files with Protobuf included.
  * @author Arne Moos
 **/

#pragma once
#include <string>
#include <time.h>

#include "Tools/Module/Module.h"
#include "Tools/ColorModelConversions.h"
#include "Tools/Protobuf/ProtobufTools.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif

#include "Tools/ImageProcessing/stb_image.h"
#include "Tools/ImageProcessing/stb_image_write.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include "Platform/File.h"

#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/PNGImage.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

STREAMABLE(ImageWriterConfig,,
  (bool)(false) enabled,                                   // bool to determine if the PNG-Logger is enabled or not
  (bool)(true) upper,                                     // log the upper image (if one of the other log options is enabled)
  (bool)(true) lower,                                     // log the lower image (if one of the other log options is enabled)
  (bool)(true) enableProtobuf                            // writes Protobuf informations if true
);

MODULE(ImageWriterPNG,
  //REQUIRES(GroundTruthWorldState),
  REQUIRES(Image),
  REQUIRES(ImageUpper),
  REQUIRES(SequenceImage),
  REQUIRES(SequenceImageUpper),
  REQUIRES(BallPercept),
  REQUIRES(MultipleBallPercept),
  REQUIRES(ProcessedBallPatches),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraIntrinsics),
  REQUIRES(BehaviorData),
  PROVIDES(PNGImageDummy),

  LOADS_PARAMETERS(,
    (ImageWriterConfig) imageConfig,
    (ImageWriterConfig) sequenceImageConfig,
    (ImageWriterConfig) ballPerceptConfig,
    (ImageWriterConfig) processedballPatchesConfig,
    (bool) useGroundTruthData,                        // use the data from the GroundTruthWorldState
    (std::string) datasetName,                        // datasetName for Protobuf
    (std::vector<std::string>) datasetNameBlacklist  // datasetsName string which should not be included in the logged name
  )
);


class ImageWriterPNG : public ImageWriterPNGBase
{
public:
  ImageWriterPNG();

private:
  std::string lastDatasetName;
  bool lastEnabledState;

  void update(PNGImageDummy& dummy);

  std::string logfile;
  std::string filepathLower, filepathUpper, filepathPatchLower, filepathPatchUpper;

  std::string getImageFilePath(bool upper);
  std::string getPatchFilePath(bool upper);

  void create_required_dirs();
  void create_subdirs(const char* path);

  int imageNumber;
  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;
  unsigned lastImageTimeStampWhenSequenceBegun;
  unsigned lastImageUpperTimeStampWhenSequenceBegun;

  void logSequenceImage(bool upper);
  void logImage(bool upper);
  void logBallPatch(const BallPatch& bp, ImageWriterConfig& config);

  std::string fillLeadingZeros(unsigned number);
  std::string theDate;
  std::string getDate();

  unsigned char targetLower[320 * 240 * 3];
  unsigned char targetUpper[640 * 480 * 3];
};
