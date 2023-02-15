#include "ImageWriterPNG.h"

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif

#include <iostream>
#include <sstream>
#include <fstream>

#include <filesystem>

ImageWriterPNG::ImageWriterPNG()
{
  theDate = getDate();

  imageNumber = 1;
  lastImageTimeStamp = 0;
  lastImageUpperTimeStamp = 0;
  lastImageTimeStampWhenSequenceBegun = 0;
  lastImageUpperTimeStampWhenSequenceBegun = 0;
  lastDatasetName = datasetName;
  lastEnabledState = imageConfig.enabled || sequenceImageConfig.enabled || ballPerceptConfig.enabled || processedballPatchesConfig.enabled;

  if (lastEnabledState)
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
      else
      {
        logfileDatasetName += token;
        logfileDatasetName.pop_back();
        logfileDatasetName.pop_back();
        logfileDatasetName.pop_back();
        logfileDatasetName.pop_back();
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
#else
  logfile = "";
#endif
}

void ImageWriterPNG::create_required_dirs()
{
  if (imageConfig.enabled || sequenceImageConfig.enabled)
  {
    filepathLower = getImageFilePath(false);
    filepathUpper = getImageFilePath(true);
    std::filesystem::create_directories(filepathLower);
    std::filesystem::create_directories(filepathUpper);
  }

  if (ballPerceptConfig.enabled || processedballPatchesConfig.enabled)
  {
    filepathPatchLower = getPatchFilePath(false);
    filepathPatchUpper = getPatchFilePath(true);
    std::filesystem::create_directories(filepathPatchLower);
    std::filesystem::create_directories(filepathPatchUpper);
  }
}

void ImageWriterPNG::update(PNGImageDummy& dummy)
{
  bool enabled = imageConfig.enabled || sequenceImageConfig.enabled || ballPerceptConfig.enabled || processedballPatchesConfig.enabled;
  if ((lastEnabledState != enabled && enabled == true) || (lastDatasetName != datasetName && datasetName != ""))
  {
    lastEnabledState = enabled;
    lastDatasetName = datasetName;
    create_required_dirs();
  }

  if (sequenceImageConfig.enabled)
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

    if (sequenceImageConfig.upper && lastImageUpperTimeStamp != theSequenceImageUpper.image.timeStamp && theSequenceImageUpper.noInSequence > 0)
    {
      lastImageUpperTimeStamp = theSequenceImageUpper.image.timeStamp;
      logSequenceImage(true);
      dummy.successful = true;
    }

    if (sequenceImageConfig.lower && lastImageTimeStamp != theSequenceImage.image.timeStamp && theSequenceImage.noInSequence > 0)
    {
      lastImageTimeStamp = theSequenceImage.image.timeStamp;
      logSequenceImage(false);
      dummy.successful = true;
    }
  }

  if (imageConfig.enabled)
  {
    dummy.successful = false;

    if (imageConfig.upper && lastImageUpperTimeStamp != theImageUpper.timeStamp)
    {
      lastImageUpperTimeStamp = theImageUpper.timeStamp;
      logImage(true);
      dummy.successful = true;
    }

    if (imageConfig.lower && lastImageTimeStamp != theImage.timeStamp)
    {
      lastImageTimeStamp = theImage.timeStamp;
      logImage(false);
      dummy.successful = true;
    }
  }

  if (ballPerceptConfig.enabled)
  {
    dummy.successful = false;
    logBallPatch(theBallPercept.ballPatch, ballPerceptConfig);

    for (BallPercept bp : theMultipleBallPercept.balls)
      logBallPatch(bp.ballPatch, ballPerceptConfig);
    dummy.successful = true;
  }

  if (processedballPatchesConfig.enabled)
  {
    dummy.successful = false;
    for (BallPatch p : theProcessedBallPatches.patches)
    {
      logBallPatch(p, processedballPatchesConfig);
    }
    dummy.successful = true;
  }
}

std::string ImageWriterPNG::fillLeadingZeros(unsigned number)
{
  char buff[5];
  snprintf(buff, sizeof(buff), "%03d", number);
  return buff;
}

std::string ImageWriterPNG::getImageFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/") + (upper ? "upper/" : "lower/");
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/") + (lastDatasetName != "" ? lastDatasetName : "") + "/" + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

std::string ImageWriterPNG::getPatchFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/patch/");
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/Patches/") + (lastDatasetName != "" ? lastDatasetName + "/" : "") + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

void ImageWriterPNG::logSequenceImage(bool upper)
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

  if (logfile != "" && lastDatasetName == "")
  {
    lastDatasetName = logfile;
    OUTPUT_TEXT("Save PNGs with dataset name: " << lastDatasetName);
    create_required_dirs();
  }

  if (lastDatasetName != "")
  {
    qualifier = lastDatasetName + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }
  else
  {
    qualifier = theDate + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }

  filepath = (upper ? filepathUpper : filepathLower) + qualifier + "/";
  if (noInSequence == 1)
  {
    std::filesystem::create_directories(filepath);
  }

  filename = filepath + qualifier + "_" + (upper ? "upper" : "lower") + "-" + fillLeadingZeros(noInSequence) + ".png";
  const char* filename_cstr = filename.c_str();

  if (filename.length() >= 255)
    OUTPUT_TEXT("Error, filename is too long: " << filename_cstr);

  std::string data = "";
  if (sequenceImageConfig.enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, lastDatasetName, (int)theBehaviorData.role, 3, (upper ? theSequenceImageUpper.image : theSequenceImage.image), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
      OUTPUT_TEXT("Error writing PNG: " << filename_cstr);
  }
}

void ImageWriterPNG::logImage(bool upper)
{
  unsigned char* imagedata = upper ? (unsigned char*)theImageUpper.image : (unsigned char*)theImage.image;
  unsigned width = upper ? theImageUpper.width : theImage.width;
  unsigned height = upper ? theImageUpper.height : theImage.height;

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

  if (logfile != "" && lastDatasetName == "")
  {
    lastDatasetName = logfile;
    OUTPUT_TEXT("Save PNGs with dataset name: " << lastDatasetName);
    create_required_dirs();
  }

  if (lastDatasetName != "")
  {
    qualifier = lastDatasetName + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp));
    // Fallback to enumerate images if timestamp is always 0
    if (std::min(theImageUpper.timeStamp, theImage.timeStamp) == 0)
    {
      qualifier = lastDatasetName + "_" + std::to_string(imageNumber);
      imageNumber = imageNumber + 1;
    }
  }
  else
  {
    qualifier = theDate + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp));
    // Fallback to enumerate images if timestamp is always 0
    if (std::min(theImageUpper.timeStamp, theImage.timeStamp) == 0)
    {
      qualifier = theDate + "_" + std::to_string(imageNumber);
      imageNumber = imageNumber + 1;
    }
  }

  filepath = (upper ? filepathUpper : filepathLower) + qualifier + "/";
  filename = filepath + qualifier + "_" + (upper ? "upper" : "lower") + ".png";
  const char* filename_cstr = filename.c_str();

  if (filename.length() >= 255)
    OUTPUT_TEXT("Error, filename is too long: " << filename_cstr);

  std::string data = "";
  if (imageConfig.enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, lastDatasetName, (int)theBehaviorData.role, 3, (upper ? theImageUpper : theImage), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
      OUTPUT_TEXT("Error writing PNG: " << filename_cstr);
  }
}

void ImageWriterPNG::logBallPatch(const BallPatch& bp, ImageWriterConfig& config)
{
  static int counter = 0;

  if (!(bp.fromUpper && config.upper) && !(!bp.fromUpper && config.lower))
    return;

  if (bp.validity == 0.0f)
    return;

  if (bp.patch.size() != (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3) || bp.patch.empty())
    return;

  const Image& image = bp.fromUpper ? (Image&)theImageUpper : theImage;
  std::string dataSetName = "LoggedBallPatchNao";

  std::string filename, filepath;

  if (logfile != "" && lastDatasetName == "")
  {
    lastDatasetName = logfile;
    OUTPUT_TEXT("Save PNGs with dataset name: " << lastDatasetName);
    create_required_dirs();
  }

  filepath = getPatchFilePath(bp.fromUpper);

  std::string data = "";
  if (config.enableProtobuf)
  {
    imageLabelData::ImageLabelData imageLabelData = ProtobufTools::fillProtobufData(
        bp.fromUpper, dataSetName, -1, 3, (bp.fromUpper ? theImageUpper : theImage), (bp.fromUpper ? theCameraMatrixUpper : theCameraMatrix), (bp.fromUpper ? theCameraIntrinsics : theCameraIntrinsics));

    //Vector2f posOnField;
    //Geometry::Circle expectedCircle;
    //if (Transformation::imageToRobotHorizontalPlane(ballSpot.position.cast<float>(), theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField)
    //  && Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle))
    //{
    //  ballSpot.radiusInImage = expectedCircle.radius;
    //  return true;
    //}

    imageLabelData::ImageLabelData::Point* upper_left_ball = new imageLabelData::ImageLabelData::Point;
    imageLabelData::ImageLabelData::Point* lower_right_ball = new imageLabelData::ImageLabelData::Point;
    upper_left_ball->set_x(static_cast<int>(bp.centerInPatch.x() - bp.radiusInPatch + 0.5));
    upper_left_ball->set_y(static_cast<int>(bp.centerInPatch.y() - bp.radiusInPatch + 0.5));
    lower_right_ball->set_x(static_cast<int>(bp.centerInPatch.x() + bp.radiusInPatch + 0.5));
    lower_right_ball->set_y(static_cast<int>(bp.centerInPatch.y() + bp.radiusInPatch + 0.5));

    imageLabelData::ImageLabelData::BoundingBox* ball_bbox = new imageLabelData::ImageLabelData::BoundingBox;
    ball_bbox->set_allocated_upperleft(upper_left_ball);
    ball_bbox->set_allocated_lowerright(lower_right_ball);
    imageLabelData::ImageLabelData::Label* ball_label = new imageLabelData::ImageLabelData::Label;
    ball_label->set_allocated_boundingbox(ball_bbox);

    std::string person = "";
    person += BallPatch::getName(bp.source);
    person += +"_";
    person += BallPatch::getName(bp.verifier);

    ball_label->set_person(person);
    ball_label->set_concealed(false);
    ball_label->set_blurriness(0);
    ball_label->set_visibilitylevel(imageLabelData::ImageLabelData_Label_VisibilityLevel_FULL); // TODO

    imageLabelData::ImageLabelData::Ball* ball_annotation = imageLabelData.add_balls();
    ball_annotation->set_moving(false);
    ball_annotation->set_allocated_label(ball_label);

    data = ProtobufTools::serializeProtobufData(imageLabelData);
  }

  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();
  int serializedImageLabelData_size = static_cast<int>(data.size());

  filename = filepath + std::to_string(bp.verifier) + "_" + std::to_string(static_cast<int>(bp.validity * 100.f)) + "%_" + std::to_string(image.timeStamp) + "_"
      + (bp.fromUpper ? "upper" : "lower") + "-" + std::to_string(counter) + ".png";
  counter++;

  std::vector<float> patch = bp.patch;
  std::transform(patch.begin(), patch.end(), patch.begin(), std::bind(std::multiplies<float>(), std::placeholders::_1, 255.f));
  std::vector<unsigned char> charPatch(patch.begin(), patch.end());

  if (!stbi_write_png(filename.c_str(), CNN_POSITION_SIZE, CNN_POSITION_SIZE, 3, &charPatch[0], CNN_POSITION_SIZE * 3, serializedImageLabelData, serializedImageLabelData_size))
  {

    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename.c_str(), CNN_POSITION_SIZE, CNN_POSITION_SIZE, 3, &charPatch[0], CNN_POSITION_SIZE * 3, serializedImageLabelData, serializedImageLabelData_size))
    {
      OUTPUT_TEXT("Error writing PNG: " << filename.c_str());
    }
  }
}

std::string ImageWriterPNG::getDate()
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

MAKE_MODULE(ImageWriterPNG, cognitionInfrastructure)
