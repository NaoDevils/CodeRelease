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

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/SequenceImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/RobotsPercept.h"
#include "Representations/Perception/PenaltyCrossHypotheses.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/PNGImage.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#define ROBOT_IMAGE_WIDTH 100
#define ROBOT_IMAGE_HEIGHT 255
#define ROBOT_IMAGE_SIZE ROBOT_IMAGE_WIDTH* ROBOT_IMAGE_HEIGHT

#define PENALTY_CROSS_PATCH_SIZE 32

STREAMABLE(ImageWriterConfig,,
  (bool)(false) enabled,                                  // bool to determine if the PNG-Logger is enabled or not
  (bool)(true) upper,                                     // log the upper image (if one of the other log options is enabled)
  (bool)(true) lower,                                     // log the lower image (if one of the other log options is enabled)
  (bool)(true) enableProtobuf                             // writes Protobuf informations if true
);

STREAMABLE(ImageWriterConfigProjectable,,
  (bool)(false) enabled,                                  // bool to determine if the PNG-Logger is enabled or not
  (bool)(true) upper,                                     // log the upper image (if one of the other log options is enabled)
  (bool)(true) lower,                                     // log the lower image (if one of the other log options is enabled)
  (bool)(true) enableProtobuf,                            // writes Protobuf informations if true
  (bool)(true) enableProjection,                          // handles out of bounds patches by projecting them back into the image
  (float)(0.0) margin                                     // the fraction of the width around the patch that will be saved
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
  //REQUIRES(RobotsPerceptTeam),
  REQUIRES(RobotsPercept),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraIntrinsics),
  REQUIRES(CameraInfoUpper),
  REQUIRES(CameraInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(BehaviorData),
  REQUIRES(PenaltyCrossHypotheses),
  REQUIRES(PenaltyCrossPercept),
  PROVIDES(PNGImageDummy),

  LOADS_PARAMETERS(,
    (ImageWriterConfig) ballPerceptConfig,
    (ImageWriterConfigProjectable) robotsPerceptConfig,
    (ImageWriterConfig) imageConfig,
    (ImageWriterConfig) sequenceImageConfig,
    (ImageWriterConfig) processedballPatchesConfig,
    (ImageWriterConfigProjectable) penaltyCrossPatchesConfig,
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
  void update(PNGImageDummy& dummy);

  int counter = 0;

  std::string logfile;
  std::string lastDatasetName;

  std::string getImageFilePath(bool upper);
  std::string getBallPatchFilePath(bool upper);
  std::string getRobotPatchFilePath(bool upper);
  std::string getPenaltyCrossPatchFilePath(bool upper);

  int imageNumber;
  unsigned lastImageTimeStamp;
  unsigned lastImageUpperTimeStamp;
  unsigned lastImageTimeStampWhenSequenceBegun;
  unsigned lastImageUpperTimeStampWhenSequenceBegun;

  void logSequenceImage(bool upper);
  void logImage(bool upper);
  void logBallPatch(const BallPatch& bp, ImageWriterConfig& config);
  void logRobotPatch(const RobotEstimate& re, ImageWriterConfigProjectable& config);
  void logPenaltyCrossPatch(const PenaltyCross& pc, ImageWriterConfigProjectable& config);

  void getUpperImageCoordinates(const RobotEstimate& re, int& upperLeftX, int& upperLeftY, int& lowerRightX, int& lowerRightY);
  std::string fillLeadingZeros(unsigned number);
  std::string theDate;
  std::string getDate();

  unsigned char targetLower[320 * 240 * 3];
  unsigned char targetUpper[640 * 480 * 3];

  std::vector<unsigned char> robotEstimateImage;
  std::vector<unsigned char> penaltyCrossPatch;
};
