#include "ImageWriterPNG.h"
#include "Representations/Perception/RobotsPercept.h"

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
  lastDatasetName = "";

#ifdef TARGET_SIM
  logfile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();

  std::string logfileDatasetName = "";

  std::istringstream ss(logfile);
  std::vector<std::string> tokens;
  std::string token;
  // collect path folders
  while (std::getline(ss, token, '/'))
  {
    tokens.push_back(token);
  }
  // use game, robot name, logfile to create datasets name
  for (size_t i = tokens.size() - 4; i < tokens.size(); i++)
  {
    token = tokens.at(i);
    if (token.find(":") != std::string::npos)
    {
      continue;
    }
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

  penaltyCrossPatch.resize(PENALTY_CROSS_PATCH_SIZE * PENALTY_CROSS_PATCH_SIZE * 3, 0);
  robotEstimateImage.resize(ROBOT_IMAGE_WIDTH * ROBOT_IMAGE_HEIGHT * 3, 0);
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

std::string ImageWriterPNG::getImageFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/image/") + (upper ? "upper/" : "lower/");
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/Images/") + (datasetName != "" ? datasetName + "/" : "") + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

std::string ImageWriterPNG::getBallPatchFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/patches/ball/") + (upper ? "upper/" : "lower/");
  ;
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/Patches/Ball/") + (datasetName != "" ? datasetName + "/" : "") + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

std::string ImageWriterPNG::getRobotPatchFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/patches/robot/") + (upper ? "upper/" : "lower/");
  ;
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/Patches/Robot/") + (datasetName != "" ? datasetName + "/" : "") + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

std::string ImageWriterPNG::getPenaltyCrossPatchFilePath(bool upper)
{
#ifdef TARGET_ROBOT
  return std::string("/home/nao/pnglogs/patches/penaltycross/") + (upper ? "upper/" : "lower/");
  ;
#else
  return File::getBHDir() + std::string("/Config/Logs/PNGs/Patches/PenaltyCross/") + (datasetName != "" ? datasetName + "/" : "") + (upper ? std::string("Upper/") : std::string("Lower/"));
#endif
}

void ImageWriterPNG::update(PNGImageDummy& dummy)
{
  counter = 0;

  if (theBehaviorData.soccerState == BehaviorData::SoccerState::penalized)
  {
    return;
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

  if (robotsPerceptConfig.enabled)
  {
    dummy.successful = false;
    for (RobotEstimate re : theRobotsPercept.robots)
    {
      logRobotPatch(re, robotsPerceptConfig);
    }
    dummy.successful = true;
  }

  if (penaltyCrossPatchesConfig.enabled)
  {
    dummy.successful = false;
    for (PenaltyCross pc : thePenaltyCrossHypotheses.penaltyCrosses)
    {
      logPenaltyCrossPatch(pc, penaltyCrossPatchesConfig);
    }
    for (PenaltyCross pc : thePenaltyCrossHypotheses.penaltyCrossesUpper)
    {
      logPenaltyCrossPatch(pc, penaltyCrossPatchesConfig);
    }

    if (thePenaltyCrossPercept.penaltyCrossWasSeen)
    {
      PenaltyCross pc;
      pc.positionInImage = thePenaltyCrossPercept.pointInImage.cast<float>();
      pc.fromUpper = thePenaltyCrossPercept.fromUpper;
      pc.validity = 1.0;
      logPenaltyCrossPatch(pc, penaltyCrossPatchesConfig);
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

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    if (datasetName != lastDatasetName)
      OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    lastDatasetName = datasetName;
  }

  if (datasetName != "")
  {
    qualifier = datasetName + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }
  else
  {
    qualifier = theDate + "_" + std::to_string(std::min(lastImageTimeStampWhenSequenceBegun, lastImageUpperTimeStampWhenSequenceBegun));
  }

  filepath = getImageFilePath(upper) + qualifier + "/";
  if (noInSequence == 1)
  {
    std::filesystem::create_directories(filepath);
  }
  filename = filepath + qualifier + "_" + (upper ? "upper" : "lower") + "-" + fillLeadingZeros(noInSequence) + ".png";
  const char* filename_cstr = filename.c_str();

  if (filename.length() >= 255)
    OUTPUT_ERROR("Error, filename is too long: " << filename_cstr);

  std::string data = "";
  if (sequenceImageConfig.enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 3, (upper ? theSequenceImageUpper.image : theSequenceImage.image), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
      OUTPUT_ERROR("Error writing PNG: " << filename_cstr);
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

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    if (datasetName != lastDatasetName)
      OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    lastDatasetName = datasetName;
  }

  if (datasetName != "")
  {
    qualifier = datasetName + "_" + std::to_string(std::min(theImageUpper.timeStamp, theImage.timeStamp));
    // Fallback to enumerate images if timestamp is always 0
    if (std::min(theImageUpper.timeStamp, theImage.timeStamp) == 0)
    {
      qualifier = datasetName + "_" + std::to_string(imageNumber);
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

  filepath = getImageFilePath(upper);
  filename = filepath + qualifier + "_" + (upper ? "upper" : "lower") + ".png";
  const char* filename_cstr = filename.c_str();

  if (filename.length() >= 255)
    OUTPUT_ERROR("Error, filename is too long: " << filename_cstr);

  std::string data = "";
  if (imageConfig.enableProtobuf)
  {
    data = ProtobufTools::serializeProtobufData(ProtobufTools::fillProtobufData(
        upper, datasetName, (int)theBehaviorData.role, 3, (upper ? theImageUpper : theImage), (upper ? theCameraMatrixUpper : theCameraMatrix), (upper ? theCameraIntrinsics : theCameraIntrinsics)));
  }
  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();

  int serializedImageLabelData_size = static_cast<int>(data.size());
  if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename_cstr, width, height, 3, upper ? targetUpper : targetLower, width * 3, serializedImageLabelData, serializedImageLabelData_size))
      OUTPUT_ERROR("Error writing PNG: " << filename_cstr);
  }
}

void ImageWriterPNG::logBallPatch(const BallPatch& bp, ImageWriterConfig& config)
{
  if (!(bp.fromUpper && config.upper) && !(!bp.fromUpper && config.lower))
    return;

  if (bp.validity == 0.0f)
    return;

  if (bp.getPatch().size() != (CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3) || bp.getPatch().empty())
    return;

  const Image& image = bp.fromUpper ? (Image&)theImageUpper : theImage;
  std::string protobufDatasetName = "LoggedBallPatchNao";

  std::string filename, filepath;

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    if (datasetName != lastDatasetName)
      OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    lastDatasetName = datasetName;
  }

  std::string data = "";
  if (config.enableProtobuf)
  {
    imageLabelData::ImageLabelData imageLabelData = ProtobufTools::fillProtobufData(
        bp.fromUpper, protobufDatasetName, -1, 3, (bp.fromUpper ? theImageUpper : theImage), (bp.fromUpper ? theCameraMatrixUpper : theCameraMatrix), (bp.fromUpper ? theCameraIntrinsics : theCameraIntrinsics));

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
    person += CheckedBallSpot::getName(bp.source);
    person += +"_";
    person += CheckedBallSpot::getName(bp.verifier);

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

  filepath = getBallPatchFilePath(bp.fromUpper);
  filename = filepath + std::to_string(bp.verifier) + "_" + std::to_string(static_cast<int>(bp.validity * 100.f)) + "%_" + std::to_string(image.timeStamp) + "_"
      + (bp.fromUpper ? "upper" : "lower") + "-" + std::to_string(counter) + ".png";
  if (filename.length() >= 255)
    OUTPUT_ERROR("Error, filename is too long: " << filename.c_str());

  counter++;

  //std::vector<float> patch = bp.patch;
  //std::transform(patch.begin(), patch.end(), patch.begin(), std::bind(std::multiplies<float>(), std::placeholders::_1, 255.f));
  //std::vector<unsigned char> charPatch(patch.begin(), patch.end());

  if (!stbi_write_png(filename.c_str(), CNN_POSITION_SIZE, CNN_POSITION_SIZE, 3, bp.getPatch().data(), CNN_POSITION_SIZE * 3, serializedImageLabelData, serializedImageLabelData_size))
  {

    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename.c_str(), CNN_POSITION_SIZE, CNN_POSITION_SIZE, 3, bp.getPatch().data(), CNN_POSITION_SIZE * 3, serializedImageLabelData, serializedImageLabelData_size))
    {
      OUTPUT_ERROR("Error writing PNG: " << filename.c_str());
    }
  }
}

void ImageWriterPNG::logRobotPatch(const RobotEstimate& re, ImageWriterConfigProjectable& config)
{
  if (!(re.fromUpperImage && config.upper) && !(!re.fromUpperImage && config.lower))
    return;

  if (re.validity == 0.0f)
    return;

  const Image& image = re.fromUpperImage ? (Image&)theImageUpper : theImage;
  std::string protobufDatasetName = "LoggedRobotEstimateNao";

  std::string filename, filepath;

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    if (datasetName != lastDatasetName)
      OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    lastDatasetName = datasetName;
  }

  std::string data = "";
  if (config.enableProtobuf)
  {
    imageLabelData::ImageLabelData imageLabelData = ProtobufTools::fillProtobufData(
        re.fromUpperImage, protobufDatasetName, -1, 3, (re.fromUpperImage ? theImageUpper : theImage), (re.fromUpperImage ? theCameraMatrixUpper : theCameraMatrix), (re.fromUpperImage ? theCameraIntrinsics : theCameraIntrinsics));

    imageLabelData::ImageLabelData::Point* upper_left_robot = new imageLabelData::ImageLabelData::Point;
    imageLabelData::ImageLabelData::Point* lower_right_robot = new imageLabelData::ImageLabelData::Point;
    upper_left_robot->set_x(static_cast<int>(re.imageUpperLeft.x()));
    upper_left_robot->set_y(static_cast<int>(re.imageUpperLeft.y()));
    lower_right_robot->set_x(static_cast<int>(re.imageLowerRight.x()));
    lower_right_robot->set_y(static_cast<int>(re.imageLowerRight.y()));

    imageLabelData::ImageLabelData::BoundingBox* robot_bbox = new imageLabelData::ImageLabelData::BoundingBox;
    robot_bbox->set_allocated_upperleft(upper_left_robot);
    robot_bbox->set_allocated_lowerright(lower_right_robot);
    imageLabelData::ImageLabelData::Label* robot_label = new imageLabelData::ImageLabelData::Label;
    robot_label->set_allocated_boundingbox(robot_bbox);

    robot_label->set_person("");
    robot_label->set_concealed(false);
    robot_label->set_blurriness(0);
    robot_label->set_visibilitylevel(imageLabelData::ImageLabelData_Label_VisibilityLevel_FULL); // TODO

    imageLabelData::ImageLabelData::Robot* robot_annotation = imageLabelData.add_robots();
    robot_annotation->set_allocated_label(robot_label);
    //TODO: color, orientation etc

    data = ProtobufTools::serializeProtobufData(imageLabelData);
  }

  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();
  int serializedImageLabelData_size = static_cast<int>(data.size());

  filepath = getRobotPatchFilePath(re.fromUpperImage);
  filename = filepath + std::to_string(static_cast<int>(re.validity * 100.f)) + "%_" + std::to_string(image.timeStamp) + "_" + (re.fromUpperImage ? "upper" : "lower") + "-"
      + std::to_string(counter) + ".png";


  const Vector2i reSize = re.imageLowerRight - re.imageUpperLeft;
  const Vector2i margin = (reSize.cast<float>() * config.margin + Vector2f::Constant(0.5f)).cast<int>();

  Vector2i min = re.imageUpperLeft - margin;
  const Vector2i size = reSize + 2 * margin;

  bool intersectsImageBoundaries = image.isOutOfImage(re.imageUpperLeft.x(), re.imageUpperLeft.y(), 0) || image.isOutOfImage(re.imageLowerRight.x(), re.imageLowerRight.y(), 0);

  if (intersectsImageBoundaries)
  {
    filepath = filepath + "intersecting/";
    if (config.enableProjection)
    {
      if (image.projectIntoImage(min.x(), min.y(), size.x(), size.y()))
      {
        filepath = filepath + "projectable/";
      }
      else
      {
        filepath = filepath + "nonProjectable/";
      }
    }
  }

  filename = filepath + std::to_string(static_cast<int>(re.validity * 100.f)) + "%_" + std::to_string(image.timeStamp) + "_" + (re.fromUpperImage ? "upper" : "lower") + "-"
      + std::to_string(counter) + ".png";
  if (filename.length() >= 255)
    OUTPUT_ERROR("Error, filename is too long: " << filename.c_str());
  counter++;

  if (re.patch.size() != (ROBOT_IMAGE_WIDTH * ROBOT_IMAGE_HEIGHT * 3) || re.patch.empty())
  {
    if (!re.fromUpperImage && re.imageUpperLeft.y() < 0)
    {
      theImage.copyAndResizeArea(min, size, {ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT}, robotEstimateImage.data());
      Vector2i ul = re.imageUpperLeft;
      Vector2i lr = re.imageLowerRight;
      getUpperImageCoordinates(re, ul.x(), ul.y(), lr.x(), lr.y());
      theImageUpper.copyAndResizeArea<true, true, false>(ul, lr - ul, {ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT}, robotEstimateImage.data());
    }
    else
    {
      image.copyAndResizeArea(min, size, {ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT}, robotEstimateImage.data());
    }
  }
  else
  {
    this->robotEstimateImage = re.patch;
  }
  std::vector<unsigned char> charPatch(robotEstimateImage.begin(), robotEstimateImage.end());

  if (!stbi_write_png(filename.c_str(), ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT, 3, &charPatch[0], 0, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename.c_str(), ROBOT_IMAGE_WIDTH, ROBOT_IMAGE_HEIGHT, 3, &charPatch[0], 0, serializedImageLabelData, serializedImageLabelData_size))
    {
      OUTPUT_ERROR("Error writing PNG: " << filename.c_str());
    }
  }
}

void ImageWriterPNG::getUpperImageCoordinates(const RobotEstimate& re, int& upperLeftX, int& upperLeftY, int& lowerRightX, int& lowerRightY)
{
  if (re.fromUpperImage)
  {
    upperLeftX = re.imageUpperLeft.x(), upperLeftY = re.imageUpperLeft.y(), lowerRightX = re.imageLowerRight.x(), lowerRightY = re.imageLowerRight.y();
  }
  float xFraction = static_cast<float>(theImageUpper.width) / theImage.width;
  float yFraction = static_cast<float>(theImageUpper.height) / theImage.height;
  upperLeftX = static_cast<int>(round(re.imageUpperLeft.x() * xFraction));
  upperLeftY = std::max(0, static_cast<int>(round(theImageUpper.height + re.imageUpperLeft.y() * yFraction)));
  lowerRightX = static_cast<int>(round(re.imageLowerRight.x() * xFraction));
  lowerRightY = static_cast<int>(round(theImageUpper.height + re.imageLowerRight.y() * yFraction));
}

void ImageWriterPNG::logPenaltyCrossPatch(const PenaltyCross& pc, ImageWriterConfigProjectable& config)
{
  if (!(pc.fromUpper && config.upper) && !(!pc.fromUpper && config.lower))
    return;

  if (pc.validity == 0.0f)
    return;

  const Image& image = pc.fromUpper ? (Image&)theImageUpper : theImage;
  const CameraMatrix& cameraMatrix = pc.fromUpper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  const CameraInfo& cameraInfo = pc.fromUpper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;


  std::string protobufDatasetName = "LoggedPenaltyCrossesNao";

  std::string filename, filepath;

  if (logfile != "" && datasetName == "")
  {
    datasetName = logfile;
    if (datasetName != lastDatasetName)
      OUTPUT_TEXT("Save PNGs with dataset name: " << datasetName);
    lastDatasetName = datasetName;
  }
  Vector2i posInImage = pc.positionInImage.cast<int>();
  float radius = Geometry::calculateLineSizePrecise(posInImage, cameraMatrix, cameraInfo, theFieldDimensions.penaltyMarkSize);
  if (config.enableProjection)
  {
    bool projected = pc.fromUpper ? theImageUpper.projectIntoImage(posInImage, radius) : theImage.projectIntoImage(posInImage, radius);
    if (!projected)
    {
      return;
    }
  }

  Vector2f upperLeft, lowerRight;
  upperLeft = posInImage.cast<float>() - Vector2f(radius, radius);
  lowerRight = posInImage.cast<float>() + Vector2f(radius, radius);

  std::string data = "";
  if (config.enableProtobuf)
  {
    imageLabelData::ImageLabelData imageLabelData = ProtobufTools::fillProtobufData(pc.fromUpper, protobufDatasetName, -1, 3, image, cameraMatrix, theCameraIntrinsics);

    imageLabelData::ImageLabelData::Label* penalty_cross_label = new imageLabelData::ImageLabelData::Label;

    imageLabelData::ImageLabelData::Point* upper_left_penalty_cross = new imageLabelData::ImageLabelData::Point;
    imageLabelData::ImageLabelData::Point* lower_right_penalty_cross = new imageLabelData::ImageLabelData::Point;
    upper_left_penalty_cross->set_x(static_cast<int>(upperLeft.x()));
    upper_left_penalty_cross->set_y(static_cast<int>(upperLeft.y()));
    lower_right_penalty_cross->set_x(static_cast<int>(lowerRight.x()));
    lower_right_penalty_cross->set_y(static_cast<int>(lowerRight.y()));
    imageLabelData::ImageLabelData::BoundingBox* penalty_cross_bbox = new imageLabelData::ImageLabelData::BoundingBox;
    penalty_cross_bbox->set_allocated_upperleft(upper_left_penalty_cross);
    penalty_cross_bbox->set_allocated_lowerright(lower_right_penalty_cross);
    penalty_cross_label->set_allocated_boundingbox(penalty_cross_bbox);
    penalty_cross_label->set_person("");
    penalty_cross_label->set_concealed(false);
    penalty_cross_label->set_blurriness(0);
    penalty_cross_label->set_visibilitylevel(imageLabelData::ImageLabelData_Label_VisibilityLevel_FULL); // TODO

    imageLabelData::ImageLabelData::PenaltyCross* penaltycross_annotation = imageLabelData.add_penaltycrosses();
    penaltycross_annotation->set_allocated_label(penalty_cross_label);
    //TODO: color, orientation etc

    data = ProtobufTools::serializeProtobufData(imageLabelData);
  }

  unsigned char* serializedImageLabelData = (unsigned char*)data.c_str();
  int serializedImageLabelData_size = static_cast<int>(data.size());

  filepath = getPenaltyCrossPatchFilePath(pc.fromUpper);
  filename = filepath + std::to_string(static_cast<int>(pc.validity * 100.f)) + "%_" + std::to_string(image.timeStamp) + "_" + (pc.fromUpper ? "upper" : "lower") + "-"
      + std::to_string(counter) + ".png";
  if (filename.length() >= 255)
    OUTPUT_ERROR("Error, filename is too long: " << filename.c_str());
  counter++;

  int xmin = static_cast<int>(upperLeft.x());
  int ymin = static_cast<int>(upperLeft.y());
  int width = static_cast<int>(lowerRight.x()) - xmin;
  int height = static_cast<int>(lowerRight.y()) - ymin;

  image.copyAndResizeArea({xmin, ymin}, {width, height}, {PENALTY_CROSS_PATCH_SIZE, PENALTY_CROSS_PATCH_SIZE}, penaltyCrossPatch.data());

  std::vector<unsigned char> charPatch(penaltyCrossPatch.begin(), penaltyCrossPatch.end());

  if (!stbi_write_png(filename.c_str(), PENALTY_CROSS_PATCH_SIZE, PENALTY_CROSS_PATCH_SIZE, 3, &charPatch[0], 0, serializedImageLabelData, serializedImageLabelData_size))
  {
    std::filesystem::create_directories(filepath);
    if (!stbi_write_png(filename.c_str(), PENALTY_CROSS_PATCH_SIZE, PENALTY_CROSS_PATCH_SIZE, 3, &charPatch[0], 0, serializedImageLabelData, serializedImageLabelData_size))
    {
      OUTPUT_ERROR("Error writing PNG: " << filename.c_str());
    }
  }
}

MAKE_MODULE(ImageWriterPNG, cognitionInfrastructure)
