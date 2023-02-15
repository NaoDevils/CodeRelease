#pragma once

#include <string>
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"

#if defined(WINDOWS)
#pragma warning(push)
#pragma warning(disable : 4146)
#include "Tools/Protobuf/imageLabelData.pb.h"
#pragma warning(pop)
#else
#include "Tools/Protobuf/imageLabelData.pb.h"
#endif


namespace ProtobufTools
{
  /**
   * @brief Calculates the average brightness of an image
   * @param upper, if current image is upper image
   * @return sum of all brightnesses for each pixel devided by the sum of all pixels
   */
  float calc_avgbrightness(const Image& image);

  /**
   * @brief Calculating variance of laplacian filter
   * Heigher values mean sharper images.
   *
   * @param upper, true if upper camera is used
   * @return Variance of Laplacian
   */
  float calc_blurness(const Image& image);

  imageLabelData::ImageLabelData fillProtobufData(
      const bool& upper, const std::string& datasetName, const int& playerNumber, const int& channel, const Image& image, const CameraMatrix& cameraMatrix, const CameraIntrinsics& cameraIntrinsics);

  /**
   * @brief serialization of image information via Protobuf
   * @return serialized data as string
   */
  std::string serializeProtobufData(const imageLabelData::ImageLabelData& imageLabelData);

} // namespace ProtobufTools
