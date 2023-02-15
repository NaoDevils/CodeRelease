#include "ProtobufTools.h"

float ProtobufTools::calc_avgbrightness(const Image& image)
{
  float sum = 0;
  for (int y = 0; y < image.height; y++)
  {
    for (int x = 0; x < image.width; x++)
    {
      sum += image[y][x].y;
    }
  }
  sum /= image.height * image.width;
  return sum;
}

float ProtobufTools::calc_blurness(const Image& image)
{
  float sum = 0;
  int conv[640 - 2][320 - 2];
  //calculation of Laplacian
  for (int y = 1; y < image.height - 1; y++)
  {
    for (int x = 1; x < image.width - 1; x++)
    {
      unsigned char b = image[y - 1][x].y;
      unsigned char d = image[y][x - 1].y;
      unsigned char e = image[y][x].y;
      unsigned char f = image[y][x + 1].y;
      unsigned char h = image[y + 1][x].y;
      int local_conv = (int)b + (int)d + (-4) * (int)e + (int)f + (int)h;
      conv[y - 1][x - 1] = local_conv;
      sum += local_conv;
    }
  }
  float mean = sum / ((image.height - 2) * (image.width - 2));

  //calculate variance
  float var = 0;
  for (int y = 0; y < image.height - 2; y++)
  {
    for (int x = 0; x < image.width - 2; x++)
    {
      var += (conv[y][x] - mean) * (conv[y][x] - mean);
    }
  }
  return var / ((image.height - 2) * (image.width - 2) - 1);
}

imageLabelData::ImageLabelData ProtobufTools::fillProtobufData(
    const bool& upper, const std::string& datasetName, const int& playerNumber, const int& channel, const Image& image, const CameraMatrix& cameraMatrix, const CameraIntrinsics& cameraIntrinsics)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  imageLabelData::ImageLabelData imageLabelData;

  //Teamname
  std::string teamname = "Nao Devils";
  imageLabelData.set_team(teamname);

  //DatasetName
  if (!datasetName.empty())
  {
    imageLabelData.set_datasetname(datasetName);
  }

  //role in team (aka number of player)
  imageLabelData.set_playernumber(playerNumber);

  //ImageInformations
  imageLabelData.mutable_imageinfos()->set_imagechannel(channel);
#ifndef TARGET_ROBOT
  imageLabelData.mutable_imageinfos()->set_blurriness(calc_blurness(image));
  imageLabelData.mutable_imageinfos()->set_average_brightnes(static_cast<int>(calc_avgbrightness(image)));
#endif

  imageLabelData.mutable_camerainformation()->mutable_resolution()->set_height(image.height);
  imageLabelData.mutable_camerainformation()->mutable_resolution()->set_width(image.width);

  //CameraIntrinsics
  if (upper)
  {
    imageLabelData.mutable_camerainformation()->set_cameraindex(0);

    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->set_openinganglewidth(cameraIntrinsics.upperOpeningAngleWidth.toDegrees());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->set_openingangleheight(cameraIntrinsics.upperOpeningAngleHeight.toDegrees());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->mutable_opticalcenter()->set_x(cameraIntrinsics.upperOpticalCenter.x());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->mutable_opticalcenter()->set_y(cameraIntrinsics.upperOpticalCenter.y());
  }
  else
  {
    imageLabelData.mutable_camerainformation()->set_cameraindex(1);

    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->set_openinganglewidth(cameraIntrinsics.lowerOpeningAngleWidth.toDegrees());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->set_openingangleheight(cameraIntrinsics.lowerOpeningAngleHeight.toDegrees());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->mutable_opticalcenter()->set_x(cameraIntrinsics.lowerOpticalCenter.x());
    imageLabelData.mutable_camerainformation()->mutable_cameraintrinsics()->mutable_opticalcenter()->set_y(cameraIntrinsics.lowerOpticalCenter.y());
  }

  //CameraMatrix
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m00(cameraMatrix.rotation(0, 0));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m01(cameraMatrix.rotation(0, 1));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m02(cameraMatrix.rotation(0, 2));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m10(cameraMatrix.rotation(1, 0));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m11(cameraMatrix.rotation(1, 1));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m12(cameraMatrix.rotation(1, 2));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m20(cameraMatrix.rotation(2, 0));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m21(cameraMatrix.rotation(2, 1));
  imageLabelData.mutable_camerainformation()->mutable_rotation()->set_m22(cameraMatrix.rotation(2, 2));
  imageLabelData.mutable_camerainformation()->mutable_translation()->set_x(cameraMatrix.translation.x());
  imageLabelData.mutable_camerainformation()->mutable_translation()->set_y(cameraMatrix.translation.y());
  imageLabelData.mutable_camerainformation()->mutable_translation()->set_z(cameraMatrix.translation.z());

  //timestamp
  imageLabelData.set_timestamp(image.timeStamp);

  return imageLabelData;
}

std::string ProtobufTools::serializeProtobufData(const imageLabelData::ImageLabelData& imageLabelData)
{
  std::string data;
  imageLabelData.SerializeToString(&data);
  return data;
}
