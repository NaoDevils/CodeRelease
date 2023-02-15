#include "PNGLogger.h"
#include <iostream>
#include <fstream>
#include <filesystem>

#ifdef WINDOWS
#include "direct.h"
#endif

PNGLogger::PNGLogger()
{
  theDate = getDate();
  imageCounter = 0;
  imageNumber = 1;

  lastImageTimeStamp = 0;
  lastImageUpperTimeStamp = 0;
  lastDatasetName = datasetName;
  lastEnabledState = enabled;

  if (enabled)
    create_required_dirs();
}

void PNGLogger::create_required_dirs()
{
  filepathLower = getFilePath(false);
  filepathUpper = getFilePath(true);

  if (std::filesystem::create_directories(filepathLower))
    OUTPUT_TEXT("Created directory " + filepathLower);
  if (std::filesystem::create_directories(filepathUpper))
    OUTPUT_TEXT("Created directory " + filepathLower);
}


void PNGLogger::update(PNGImageDummy& dummy)
{
  if ((lastEnabledState != enabled && enabled == true) || lastDatasetName != datasetName)
  {
    create_required_dirs();
  }
  lastEnabledState = enabled;
  lastDatasetName = datasetName;

  if (enabled && (imageCounter % everyXFrame == 0))
  {
    dummy.successful = false;

    if (upper && lastImageUpperTimeStamp != theImageUpper.timeStamp)
    {
      lastImageUpperTimeStamp = theImageUpper.timeStamp;
      if (logYUV)
        logImage(true);
      if (logY)
        logImageY(true);
      if (logYFull)
        logImageYFull(true);
      dummy.successful = true;
    }

    if (lower && lastImageTimeStamp != theImage.timeStamp)
    {
      lastImageTimeStamp = theImage.timeStamp;
      if (logYUV)
        logImage(false);
      if (logY)
        logImageY(false);
      if (logYFull)
        logImageYFull(false);
      dummy.successful = true;
    }

    if (dummy.successful)
      imageCounter++;
  }
}

std::string PNGLogger::fillLeadingZeros(unsigned number)
{
  char buff[5];
  snprintf(buff, sizeof(buff), "%04d", number);
  return buff;
}

std::string PNGLogger::getFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/") + (upper ? "upper/" : "lower/");
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/") + (datasetName != "" ? datasetName : "") + "/" + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

void PNGLogger::logImage(bool upper)
{
  unsigned char* imagedata = upper ? (unsigned char*)theImageUpper.image : (unsigned char*)theImage.image;
  unsigned width = upper ? theImageUpper.width : theImage.width;
  unsigned height = upper ? theImageUpper.height : theImage.height;

  unsigned char* target_ptr = upper ? targetUpper : targetLower;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      if (streamAsRGB)
      {
        unsigned char r;
        unsigned char g;
        unsigned char b;

        ColorModelConversions::fromYCbCrToRGB(imagedata[0], imagedata[1], imagedata[3], r, g, b);

        target_ptr[0] = r;
        target_ptr[1] = g;
        target_ptr[2] = b;
      }
      else
      {
        target_ptr[0] = imagedata[0];
        target_ptr[1] = imagedata[1];
        target_ptr[2] = imagedata[3];
      }


      target_ptr += 3;
      imagedata += 4;
    }
    imagedata += width * 4;
  }

  std::string filename;
  if (datasetName != "")
  {
    filename = (upper ? filepathUpper : filepathLower) + datasetName + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_" + (upper ? "upper" : "lower") + ".png";
  }
  else
  {
    filename = (upper ? filepathUpper : filepathLower) + theDate + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_" + (upper ? "upper" : "lower") + ".png";
  }
  // Fallback to enumerate images if timestamp is always 0
  if (std::min(theImageUpper.timeStamp, theImage.timeStamp) == 0)
  {
    filename = (upper ? filepathUpper : filepathLower) + datasetName + "_" + std::to_string(imageNumber) + "_" + (upper ? "upper" : "lower") + ".png";
    imageNumber = imageNumber + 1;
  }
  const char* filename_cstr = filename.c_str();

  std::string data = "";
  if (enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 3, (upper ? theImageUpper : theImage), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size);
}

void PNGLogger::logImageYFull(bool upper)
{
  unsigned char* imagedata = upper ? (unsigned char*)theImageUpper.image : (unsigned char*)theImage.image;
  unsigned width = upper ? theImageUpper.width * 2 : theImage.width * 2;
  unsigned height = upper ? theImageUpper.height * 2 : theImage.height * 2;

  unsigned char* target_ptr = upper ? targetUpperFull : targetLowerFull;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      if (streamAsRGB)
      {
        unsigned char r;
        unsigned char g;
        unsigned char b;

        ColorModelConversions::fromYCbCrToRGB(imagedata[0], 127, 127, r, g, b);

        target_ptr[0] = r;
        target_ptr[1] = g;
        target_ptr[2] = b;
      }
      else
      {
        target_ptr[0] = imagedata[0];
        target_ptr[1] = imagedata[1];
        target_ptr[2] = imagedata[3];
      }


      target_ptr += 3;
      imagedata += 2;
    }
    //imagedata+=width*4;
  }

  std::string filename;

  if (datasetName != "")
  {
    filename = (upper ? filepathUpper : filepathLower) + datasetName + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_"
        + (upper ? "upper_full_onlyY" : "lower_full_onlyY") + ".png";
  }
  else
  {
    filename = (upper ? filepathUpper : filepathLower) + theDate + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_"
        + (upper ? "upper_full_onlyY" : "lower_full_onlyY") + ".png";
  }


  const char* filename_cstr = filename.c_str();
  std::string data = "";
  if (enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 1, (upper ? theImageUpper : theImage), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();
  stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpperFull : targetLowerFull, width * 3, serializedImageLabelData);
}

void PNGLogger::logImageY(bool upper)
{
  unsigned char* imagedata = upper ? (unsigned char*)theImageUpper.image : (unsigned char*)theImage.image;
  unsigned width = upper ? theImageUpper.width : theImage.width;
  unsigned height = upper ? theImageUpper.height : theImage.height;

  unsigned char* target_ptr = upper ? targetUpper : targetLower;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      if (streamAsRGB)
      {
        unsigned char r;
        unsigned char g;
        unsigned char b;

        ColorModelConversions::fromYCbCrToRGB(imagedata[0], 127, 127, r, g, b);

        target_ptr[0] = r;
        target_ptr[1] = g;
        target_ptr[2] = b;
      }
      else
      {
        target_ptr[0] = imagedata[0];
        target_ptr[1] = imagedata[1];
        target_ptr[2] = imagedata[3];
      }


      target_ptr += 3;
      imagedata += 4;
    }
    imagedata += width * 4;
  }

  std::string filename;
  if (datasetName != "")
  {
    filename = (upper ? filepathUpper : filepathLower) + datasetName + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_"
        + (upper ? "upper_onlyY" : "lower_onlyY") + ".png";
  }
  else
  {
    filename = (upper ? filepathUpper : filepathLower) + theDate + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp)) + "_"
        + (upper ? "upper_onlyY" : "lower_onlyY") + ".png";
  }
  const char* filename_cstr = filename.c_str();
  std::string data = "";
  if (enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 1, (upper ? theImageUpper : theImage), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();
  stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData);
}

std::string PNGLogger::getDate()
{
  time_t now;
  char the_date[30];

  the_date[0] = '\0';

  now = time(NULL);

  if (now != -1)
  {
    strftime(the_date, sizeof(the_date), "%d_%m_%Y__%H_%M_%S", localtime(&now));
  }

  return std::string(the_date);
}

MAKE_MODULE(PNGLogger, cognitionInfrastructure)
