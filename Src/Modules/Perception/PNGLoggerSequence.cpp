#include "PNGLoggerSequence.h"
#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif

#include <iostream>
#include <sstream>
#include <fstream>

#include <filesystem>

PNGLoggerSequence::PNGLoggerSequence()
{
  theDate = getDate();

  lastImageTimeStamp = 0;
  lastImageUpperTimeStamp = 0;
  lastImageTimeStampWhenSequenceBegun = 0;
  lastImageUpperTimeStampWhenSequenceBegun = 0;
  lastDatasetName = datasetName;
  lastEnabledState = enabled;
  logfile = "";

  if (enabled)
    create_required_dirs();

#ifdef TARGET_SIM
  logfile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();

  std::string logfileDatasetName = "";

  std::istringstream ss(logfile);
  std::string token;

  while (std::getline(ss, token, '/'))
  {
    if (token.find(".log") != std::string::npos)
    {
      if (token.find("1st") != std::string::npos)
      {
        logfileDatasetName += "1st";
      }
      else if (token.find("2nd") != std::string::npos)
      {
        logfileDatasetName += "2nd";
      }
    }
    else if (std::find(datasetNameBlacklist.begin(), datasetNameBlacklist.end(), token) != datasetNameBlacklist.end())
    {
      continue;
    }
    else
    {
      logfileDatasetName += token + "-";
    }
  }

  if (logfileDatasetName.size() > 0 && logfileDatasetName.back() == '-')
  {
    logfileDatasetName.pop_back();
  }

  logfile = logfileDatasetName;
#endif
}

void PNGLoggerSequence::create_required_dirs()
{
  filepathLower = getFilePath(false);
  filepathUpper = getFilePath(true);

  if (std::filesystem::create_directories(filepathLower))
    OUTPUT_TEXT("Created directory " + filepathLower);
  if (std::filesystem::create_directories(filepathUpper))
    OUTPUT_TEXT("Created directory " + filepathLower);
}

void PNGLoggerSequence::update(PNGImageDummy& dummy)
{
  //DEBUG_RESPONSE_ONCE("test_all_requests")
  //{
  //  DebugRequest debugRequest;
  //  debugRequest.description = "log remove idThumbnail";
  //  debugRequest.enable = true;
  //  Global::getDebugRequestTable().addRequest(debugRequest);
  //}

  if ((lastEnabledState != enabled && enabled == true) || lastDatasetName != datasetName)
  {
    create_required_dirs();
  }
  lastEnabledState = enabled;
  lastDatasetName = datasetName;

  if (enabled)
  {
    dummy.successful = false;

    if (theSequenceImageUpper.noInSequence == 0)
      lastImageUpperTimeStampWhenSequenceBegun = 0;
    if (theSequenceImage.noInSequence == 0)
      lastImageTimeStampWhenSequenceBegun = 0;

    if (theSequenceImageUpper.noInSequence == 1 || (lastImageUpperTimeStampWhenSequenceBegun == 0 && theSequenceImageUpper.noInSequence > 1))
      lastImageUpperTimeStampWhenSequenceBegun = theSequenceImageUpper.image.timeStamp;
    if (theSequenceImage.noInSequence == 1 || (lastImageTimeStampWhenSequenceBegun == 0 && theSequenceImage.noInSequence > 1))
      lastImageTimeStampWhenSequenceBegun = theSequenceImage.image.timeStamp;

    if (upper && lastImageUpperTimeStamp != theSequenceImageUpper.image.timeStamp && theSequenceImageUpper.noInSequence > 0)
    {
      lastImageUpperTimeStamp = theSequenceImageUpper.image.timeStamp;
      logImage(true);
      dummy.successful = true;
    }

    if (lower && lastImageTimeStamp != theSequenceImage.image.timeStamp && theSequenceImage.noInSequence > 0)
    {
      lastImageTimeStamp = theSequenceImage.image.timeStamp;
      logImage(false);
      dummy.successful = true;
    }
  }
}

std::string PNGLoggerSequence::fillLeadingZeros(unsigned number)
{
  char buff[4];
  snprintf(buff, sizeof(buff), "%03d", number);
  return buff;
}

std::string PNGLoggerSequence::getFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/") + (upper ? "upper/" : "lower/");
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/") + (datasetName != "" ? datasetName : "") + "/" + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

void PNGLoggerSequence::logImage(bool upper)
{
  unsigned char* imagedata = upper ? (unsigned char*)theSequenceImageUpper.image.image : (unsigned char*)theSequenceImage.image.image;
  unsigned noInSequence = upper ? theSequenceImageUpper.noInSequence : theSequenceImage.noInSequence;
  unsigned width = upper ? theSequenceImageUpper.image.width : theSequenceImage.image.width;
  unsigned height = upper ? theSequenceImageUpper.image.height : theSequenceImage.image.height;

  unsigned char* target_ptr = upper ? targetUpper : targetLower;

  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      unsigned char r;
      unsigned char g;
      unsigned char b;

      ColorModelConversions::fromYCbCrToRGB(imagedata[0], imagedata[1], imagedata[3], r, g, b);

      target_ptr[0] = r;
      target_ptr[1] = g;
      target_ptr[2] = b;

      target_ptr += 3;
      imagedata += 4;
    }
    imagedata += width * 4;
  }

  std::string filename;
  std::string filepath;
  std::string qualifier;

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    create_required_dirs();
  }

  if (datasetName != "")
  {
    qualifier = datasetName + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }
  else
  {
    qualifier = theDate + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }

  filepath = (upper ? filepathUpper : filepathLower) + qualifier + "/";
  if (noInSequence == 1)
  {
    if (std::filesystem::create_directories(filepath))
      OUTPUT_TEXT("Created directory " + filepath);
  }

  filename = filepath + qualifier + "_" + (upper ? "upper" : "lower") + "-" + fillLeadingZeros(noInSequence) + ".png";
  const char* filename_cstr = filename.c_str();

  if (filename.length() >= 255)
    OUTPUT_TEXT("Error, filename is too long: " << filename_cstr);

  std::string data = "";
  if (enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 3, (upper ? theSequenceImageUpper.image : theSequenceImage.image), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
  {
    if (std::filesystem::create_directories(filepath))
      OUTPUT_TEXT("Created directory " + filepath);
    if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
      OUTPUT_TEXT("Error writing PNG: " << filename_cstr);
  }
}

std::string PNGLoggerSequence::getDate()
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

MAKE_MODULE(PNGLoggerSequence, cognitionInfrastructure)
