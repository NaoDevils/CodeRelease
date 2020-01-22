#include "YoloRobotDetector.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ColorModelConversions.h"

#include "Tools/Math/sse_mathfun.h"

#include "Tools/Settings.h"

#ifdef WINDOWS
  #include "direct.h"
#endif

// #define LAYER_STOPWATCH
////// UPPER /////
namespace upper {
  #include "tiny-yolo-nao-v29_seperable_6-456_framework.c"      // V6: 1.6ms -> alter Compiler
}

/////// LOWER /////
namespace lower {
  #include "tiny-yolo-nao-v34_lower-485_framework.c"            // V6: 0.6ms -> alter Compiler
}

////// UPPER V6 /////
namespace upperV6 {
  #include "tiny-yolo-nao-v29_seperable_6_V6-496_framework.c"   // V6: 5.1ms -> alter Compiler
}

/////// LOWER V6 /////
namespace lowerV6 {
  #include "tiny-yolo-nao-v34_lower_V6-499_framework.c"         // V6: 5.0ms -> alter Compiler
}

YoloRobotDetector::YoloRobotDetector()
{
  timeStamp = 0;
  timeStampUpper = 0;

  if(Global::getSettings().naoVersion == RobotConfig::V6)
  {
    yoloParameterUpper.input_height = upperV6::input_height;
    yoloParameterUpper.input_width = upperV6::input_width;
    yoloParameterUpper.input_channel = upperV6::input_channel;
    yoloParameterUpper.output_height = upperV6::output_height;
    yoloParameterUpper.output_width = upperV6::output_width;
    yoloParameterUpper.num_of_boxes = upperV6::num_of_boxes;
    yoloParameterUpper.num_of_classes = upperV6::num_of_classes;
    yoloParameterUpper.num_of_coords = upperV6::num_of_coords;
    yoloParameterUpper.anchors = upperV6::anchors;
  }
  else
  {
    yoloParameterUpper.input_height = upper::input_height;
    yoloParameterUpper.input_width = upper::input_width;
    yoloParameterUpper.input_channel = upper::input_channel;
    yoloParameterUpper.output_height = upper::output_height;
    yoloParameterUpper.output_width = upper::output_width;
    yoloParameterUpper.num_of_boxes = upper::num_of_boxes;
    yoloParameterUpper.num_of_classes = upper::num_of_classes;
    yoloParameterUpper.num_of_coords = upper::num_of_coords;
    yoloParameterUpper.anchors = upper::anchors;
  }

  detectionVectorUpper.reserve(yoloParameterUpper.output_height * yoloParameterUpper.output_width * yoloParameterUpper.num_of_boxes);
  inputVectorUpper.resize(yoloParameterUpper.input_height * yoloParameterUpper.input_width * yoloParameterUpper.input_channel, 0.f);
  localRobotsPerceptUpper.robots.clear();

  if(Global::getSettings().naoVersion == RobotConfig::V6)
  {
    yoloParameter.input_height = lowerV6::input_height;
    yoloParameter.input_width = lowerV6::input_width;
    yoloParameter.input_channel = lowerV6::input_channel;
    yoloParameter.output_height = lowerV6::output_height;
    yoloParameter.output_width = lowerV6::output_width;
    yoloParameter.num_of_boxes = lowerV6::num_of_boxes;
    yoloParameter.num_of_classes = lowerV6::num_of_classes;
    yoloParameter.num_of_coords = lowerV6::num_of_coords;
    yoloParameter.anchors = lowerV6::anchors;
  }
  else
  {
    yoloParameter.input_height = lower::input_height;
    yoloParameter.input_width = lower::input_width;
    yoloParameter.input_channel = lower::input_channel;
    yoloParameter.output_height = lower::output_height;
    yoloParameter.output_width = lower::output_width;
    yoloParameter.num_of_boxes = lower::num_of_boxes;
    yoloParameter.num_of_classes = lower::num_of_classes;
    yoloParameter.num_of_coords = lower::num_of_coords;
    yoloParameter.anchors = lower::anchors;
  }
  detectionVector.reserve(yoloParameter.output_height * yoloParameter.output_width * yoloParameter.num_of_boxes);
  inputVector.resize(yoloParameter.input_height * yoloParameter.input_width * yoloParameter.input_channel, 0.f);
  localRobotsPercept.robots.clear();
}

void YoloRobotDetector::reset(const bool& upper) {
  if (upper) 
  {
    localRobotsPerceptUpper.robots.clear();
    localBallHypotheses.ballSpotsUpper.clear();
  }
  else 
  {
    localRobotsPercept.robots.clear();
    localBallHypotheses.ballSpots.clear();
  }
}

void YoloRobotDetector::update(RobotsPercept& theRobotsPercept)
{
  DECLARE_DEBUG_DRAWING("module:YoloRobotDetector:shirtScan", "drawingOnImage");
  theRobotsPercept.robots.clear();
  
  execute(true);
  execute(false);

  addObstacleFromBumpers();

  theRobotsPercept = localRobotsPercept;
}

void YoloRobotDetector::update(RobotsPerceptUpper& theRobotsPerceptUpper)
{
  DECLARE_DEBUG_DRAWING("module:YoloRobotDetector:shirtScanUpper", "drawingOnImage");
  theRobotsPerceptUpper.robots.clear();

  execute(true);
  execute(false);

  theRobotsPerceptUpper = localRobotsPerceptUpper;
}

void YoloRobotDetector::update(BallHypothesesYolo& theBallHypothesesYolo) {
  theBallHypothesesYolo.ballSpots.clear();
  theBallHypothesesYolo.ballSpotsUpper.clear();

  execute(true);
  execute(false);

  theBallHypothesesYolo = localBallHypotheses;
}

void YoloRobotDetector::update(YoloInputUpper& theYoloInputUpper)
{
  theYoloInputUpper.image = inputVectorUpper;
  theYoloInputUpper.width = yoloParameterUpper.input_width;
  theYoloInputUpper.height = yoloParameterUpper.input_height;
  theYoloInputUpper.channel = yoloParameterUpper.input_channel;
  theYoloInputUpper.timeStamp = theImageUpper.timeStamp;
  theYoloInputUpper.imageUpdated = true;
}

void YoloRobotDetector::update(YoloInput& theYoloInput)
{
  theYoloInput.image = inputVector;
  theYoloInput.width = yoloParameter.input_width;
  theYoloInput.height = yoloParameter.input_height;
  theYoloInput.channel = yoloParameter.input_channel;
  theYoloInput.timeStamp = theImage.timeStamp;
  theYoloInput.imageUpdated = true;
}

void YoloRobotDetector::execute(const bool& upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  unsigned actualTimeStamp = upper ? timeStampUpper : timeStamp;
  if (actualTimeStamp != image.timeStamp)
  {
    actualTimeStamp = image.timeStamp;
    if (upper)
    {
      timeStampUpper = actualTimeStamp;
    }
    else
    {
      timeStamp = actualTimeStamp;
    }

    reset(upper);

    const CameraMatrix &cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    if (theFallDownState.state != FallDownState::upright || cameraMatrix.isValid == false)
      return;

    RobotsPercept &localPercepts = upper ? (RobotsPercept&)localRobotsPerceptUpper : localRobotsPercept;
    YoloParameter &localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;
    const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    std::vector<float> &input = upper ? (std::vector<float>&)inputVectorUpper : inputVector;
    std::vector<YoloDetection> &localDetectionVector = upper ? (std::vector<YoloDetection>&)detectionVectorUpper : detectionVector;
    localDetectionVector.clear();

    YoloResult result(localParameter.output_height, localParameter.output_width, localParameter.num_of_boxes, localParameter.num_of_coords, localParameter.num_of_classes);

    STOPWATCH("YOLO-Copy")
    {
      if (localParameter.input_channel == 3) {
        image.copyAndResizeAreaRGBFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, &input[0]);
      }
      else {
        image.copyAndResizeAreaFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, &input[0]);
      }
    }

    if (upper) {
      COMPLEX_IMAGE(YoloDebugImageUpper)
      {
        INIT_DEBUG_IMAGE_BLACK(YoloDebugImageUpper, yoloParameterUpper.input_width, yoloParameterUpper.input_height);
        for (unsigned i = 0; i < yoloParameterUpper.input_height; i++) {
          for (unsigned j = 0; j < yoloParameterUpper.input_width; j++) {

            if (yoloParameterUpper.input_channel == 3) {
              float pixely = input[i * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j * yoloParameterUpper.input_channel + 0] * 255.f;
              float pixelu = input[i * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j * yoloParameterUpper.input_channel + 1] * 255.f;
              float pixelv = input[i * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j * yoloParameterUpper.input_channel + 2] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_RGB(YoloDebugImageUpper, j, i, static_cast<unsigned char>(pixely), static_cast<unsigned char>(pixelu), static_cast<unsigned char>(pixelv));
            }
            else {
              float pixel = input[i * yoloParameterUpper.input_width + j] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_YUV(YoloDebugImageUpper, j, i, static_cast<unsigned char>(pixel), 127, 127);
            }

          }
        }
        SEND_DEBUG_IMAGE(YoloDebugImageUpper);
      }
    }
    else {
      COMPLEX_IMAGE(YoloDebugImage)
      {
        INIT_DEBUG_IMAGE_BLACK(YoloDebugImage, yoloParameter.input_width, yoloParameter.input_height);
        for (unsigned i = 0; i < yoloParameter.input_height; i++) {
          for (unsigned j = 0; j < yoloParameter.input_width; j++) {

            if (yoloParameter.input_channel == 3) {
              float pixely = input[i * yoloParameter.input_width * yoloParameter.input_channel + j * yoloParameter.input_channel + 0] * 255.f;
              float pixelu = input[i * yoloParameter.input_width * yoloParameter.input_channel + j * yoloParameter.input_channel + 1] * 255.f;
              float pixelv = input[i * yoloParameter.input_width * yoloParameter.input_channel + j * yoloParameter.input_channel + 2] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_RGB(YoloDebugImage, j, i, static_cast<unsigned char>(pixely), static_cast<unsigned char>(pixelu), static_cast<unsigned char>(pixelv));
            }
            else {
              float pixel = input[i * yoloParameter.input_width + j] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_YUV(YoloDebugImage, j, i, static_cast<unsigned char>(pixel), 127, 127);
            }

          }
        }
        SEND_DEBUG_IMAGE(YoloDebugImage);
      }
    }


    if (upper)
    {
      STOPWATCH("YOLO-ExecutionUpper")
      {
        if(Global::getSettings().naoVersion == RobotConfig::V6)
        {
          upperV6::cnn(input.data(), result.result.data());
        }
        else
        {
          upper::cnn(input.data(), result.result.data());
        }
      }
    }
    else {
      STOPWATCH("YOLO-Execution")
      {
        if(Global::getSettings().naoVersion == RobotConfig::V6)
        {
          lowerV6::cnn(input.data(), result.result.data());
        }
        else
        {
          lower::cnn(input.data(), result.result.data());
        }
      }
    }

    STOPWATCH("YOLO-Postprocessing")
    {
      generateNetworkBoxes(1, localDetectionVector, result, upper);
      std::sort(localDetectionVector.begin(), localDetectionVector.end());

      for (size_t i = 0; i < localDetectionVector.size(); i++) {
        if (localDetectionVector[i].prob == 0.f) {
          continue;
        }
        else {
          for (size_t j = i + 1; j < localDetectionVector.size(); j++) {
            if (iou(localDetectionVector[i].bbox, localDetectionVector[j].bbox, image.height, image.width) >= nmsThreshold) {
              localDetectionVector[j].prob = 0.f;
            }
          }
        }
      }

      for (size_t boxNo = 0; boxNo < localDetectionVector.size(); boxNo++) {
        const YoloDetection &det = localDetectionVector[boxNo];

        if (det.prob > 0.f) 
        {
          if (det.sortClass == YoloClasses::Robot) {
            //// ROBOT ////
            RobotEstimate re;
            re.locationOnField.rotation = 0;
            re.fromUpperImage = upper;
            re.validity = det.prob;
            re.timestampFromImage = image.timeStamp;

            Vector2f boxCenter(det.bbox.x * image.width, det.bbox.y * image.height);
            Vector2f vecBoxCenterToUpperLeft(-(det.bbox.w * image.width) / 2.f, -(det.bbox.h * image.height) / 2);
            re.imageUpperLeft = (boxCenter + vecBoxCenterToUpperLeft).cast<int>();
            re.imageLowerRight = (boxCenter - vecBoxCenterToUpperLeft).cast<int>();

            if (Transformation::imageToRobot(Vector2f(boxCenter.x(), re.imageLowerRight.y()), cameraMatrix, cameraInfo,
              re.locationOnField.translation) &&
              !(re.locationOnField.translation.x() < 100 && re.locationOnField.translation.y() < 2000) &&
              re.locationOnField.translation.x() > -1000) {

              re.distance = re.locationOnField.translation.norm();
              float heightInImage = 0.f;
              bool localUseYoloHeight = upper ? useYoloHeightUpper : useYoloHeight;
              if (!localUseYoloHeight) {
                heightInImage = Geometry::getSizeByDistance(cameraInfo, 550.f, re.distance);
              }
              else {
                heightInImage = det.bbox.h * image.height;
              }
              re.imageLowerRight.y() = static_cast<int>(re.imageLowerRight.y());
              re.imageLowerRight.x() = static_cast<int>(boxCenter.x() + heightInImage / 5.f);
              re.imageUpperLeft.x() = static_cast<int>(boxCenter.x() - heightInImage / 5.f);
              re.imageUpperLeft.y() = static_cast<int>(re.imageLowerRight.y() - heightInImage);
              re.robotType = scanForRobotColor(
                Vector2f(boxCenter.x() - 0.05f*(det.bbox.w * image.width), re.imageLowerRight.y() - heightInImage / 2.f),
                Vector2f(0.f, -1.f),
                (int)(2.f * heightInImage / 6.f),
                upper
              );
              if (re.robotType == RobotEstimate::unknownRobot)
              {
                re.robotType = scanForRobotColor(
                  Vector2f(boxCenter.x() + 0.05f*(det.bbox.w * image.width), re.imageLowerRight.y() - heightInImage / 2.f),
                  Vector2f(0.f, -1.f),
                  (int)(2.f * heightInImage / 6.f),
                  upper
                );
              }
              localPercepts.robots.push_back(re);
            }
          }
          else if (det.sortClass == YoloClasses::Ball)
          {
            //// BALL ////
            BallSpot bsy;
            bsy.position = Vector2f(det.bbox.x * image.width, det.bbox.y * image.height).cast<int>();
            bsy.radiusInImage = std::min<float>((det.bbox.w * image.width) / 2, (det.bbox.h * image.height) / 2);

            bool localUseYoloHeight = upper ? useYoloHeightUpper : useYoloHeight;

            if(!localUseYoloHeight)
            {
              // theoretical diameter if cameramatrix is correct
              Vector2f posOnField;
              Geometry::Circle expectedCircle;
              if(Transformation::imageToRobotHorizontalPlane(bsy.position.cast<float>(), theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField) 
                 && Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle))
              {
                bsy.radiusInImage = expectedCircle.radius;
              }
            }

            bsy.validity = det.prob;
            bsy.cb = 127;
            bsy.cr = 127;
            bsy.y = 0;
            if (upper)
            {
              localBallHypotheses.ballSpotsUpper.push_back(bsy);
            }
            else
            {
              localBallHypotheses.ballSpots.push_back(bsy);
            }
          }
        }
      }
    }
  }
}

void YoloRobotDetector::generateNetworkBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, YoloResult& yoloResult, const bool &upper)
{
  YoloParameter &localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;

  for (unsigned int i = 0; i < localParameter.output_width*localParameter.output_height; ++i)
  {
    int row = i / localParameter.output_width;
    int col = i % localParameter.output_width;
    for (unsigned int n = 0; n < localParameter.num_of_boxes; ++n)
    {
      YoloDetection newDetection;
      YoloRegionBox newBox;
      getRegionBox(
        newBox,
        yoloResult,
        n,
        col,
        row,
        localParameter
      );

      newDetection.bbox = newBox;

      float conf = newBox.conf; // object confidence calculated in getRegionBox(..)

      // calculate soft max over class probability values (they represent a prob. distribution) to find the most probable class
      float maxScore = -INFINITY;
      int bestClassIndex = 0;

      if (localParameter.num_of_classes > 1) {
        std::vector<float> classPredictions;
        int baseIndex = n*(localParameter.num_of_coords + 1 + localParameter.num_of_classes) + localParameter.num_of_coords + 1;
        for (unsigned int j = 0; j < localParameter.num_of_classes; ++j)
        {
          classPredictions.push_back(yoloResult(row, col, baseIndex + j));
        }
        softmax<float>(&classPredictions[0], localParameter.num_of_classes);
        for (unsigned int j = 0; j < localParameter.num_of_classes; ++j)
        {
          if (classPredictions[j] > maxScore)
          {
            maxScore = classPredictions[j];
            bestClassIndex = j;
          }
        }
      }
      else { // Special Conditions - only one class
        maxScore = 1;
        bestClassIndex = 0;
      }

      float max_class_conf = conf*maxScore;
      if ((bestClassIndex == YoloClasses::Robot && max_class_conf > (upper ? detectionThresholdUpper : detectionThreshold)) || 
          (bestClassIndex == YoloClasses::Ball && max_class_conf > (upper ? detectionThresholdUpperBall : detectionThresholdBall)))
      {
        newDetection.prob = max_class_conf;
        newDetection.classes = localParameter.num_of_classes;
        newDetection.sortClass = bestClassIndex;
        localDetectionVector.push_back(newDetection);
      }
    }
  }
  correctRegionBoxes(relative, localDetectionVector, upper);
}

void YoloRobotDetector::getRegionBox(YoloRegionBox &b,
  YoloResult& yoloResult, int numOfBox,
  int col, int row, 
  const YoloParameter &localParameter
)
{
  int baseIndex = numOfBox * (localParameter.num_of_coords + 1 + localParameter.num_of_classes);
  alignas(16) float input[4] = { -yoloResult(row,col,baseIndex), -yoloResult(row,col,baseIndex+1), yoloResult(row,col,baseIndex+2), yoloResult(row,col,baseIndex+3) };
  alignas(16) float result[4];
  _mm_store_ps(result, exp_ps(_mm_load_ps(&input[0])));

  // all values get absolute by dividing through width (height) -> get image coordinates by multiplying with image width (height) later
  b.x = (col + 1 / (1 + result[0])) / localParameter.output_width; // paper: c_x + sigmoid t_x
  b.y = (row + 1 / (1 + result[1])) / localParameter.output_height; // paper: c_y + sigmoid t_y
  b.w = result[2] * localParameter.anchors[2 * numOfBox] / localParameter.output_width; // paper: p_w * e^(t_w) with p_w being anchor w
  b.h = result[3] * localParameter.anchors[2 * numOfBox + 1] / localParameter.output_height; // paper: p_h * e^(t_h) with p_h being anchor h
  b.conf = 1 / (1 + std::exp(-yoloResult(row, col, baseIndex + 4))); // paper: sigmoid of (t_o) with t_o being object confidence
  b.grid_x = col;
  b.grid_y = row;
}

void YoloRobotDetector::correctRegionBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, const bool &upper)
{
  YoloParameter &localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;

  int new_w = 0;
  int new_h = 0;
  if (((float)localParameter.input_width / localParameter.output_width) < ((float)localParameter.input_height / localParameter.output_height)) {
    new_w = localParameter.input_width;
    new_h = (localParameter.output_height * localParameter.input_width) / localParameter.output_width;
  }
  else {
    new_h = localParameter.input_height;
    new_w = (localParameter.output_width * localParameter.input_height) / localParameter.output_height;
  }

  for (std::vector<YoloDetection>::iterator it = localDetectionVector.begin(); it != localDetectionVector.end(); ++it)
  {
    (*it).bbox.x = ((*it).bbox.x - (float)((localParameter.input_width - new_w) / 2. / localParameter.input_width)) / ((float)new_w / localParameter.input_width);
    (*it).bbox.y = ((*it).bbox.y - (float)((localParameter.input_height - new_h) / 2. / localParameter.input_height)) / ((float)new_h / localParameter.input_height);
    (*it).bbox.w *= (float)localParameter.input_width / new_w;
    (*it).bbox.h *= (float)localParameter.input_height / new_h;
  }
}

float YoloRobotDetector::iou(YoloRegionBox &box1, YoloRegionBox &box2, int heigth, int width) {
  float lowerRightX1 = box1.x * width + (box1.w * width / 2.f);
  float lowerRightX2 = box2.x * width + (box2.w * width / 2.f);
  float upperLeftX1 = box1.x * width - (box1.w * width / 2.f);
  float upperLeftX2 = box2.x * width - (box2.w * width / 2.f);

  float lowerRightY1 = box1.y * heigth + (box1.h * heigth / 2.f);
  float lowerRightY2 = box2.y * heigth + (box2.h * heigth / 2.f);
  float upperLeftY1 = box1.y * heigth - (box1.h * heigth / 2.f);
  float upperLeftY2 = box2.y * heigth - (box2.h * heigth / 2.f);

  float intersection = std::max(0.f, std::min(lowerRightX1, lowerRightX2) - std::max(upperLeftX1, upperLeftX2)) *
    std::max(0.f, std::min(lowerRightY1, lowerRightY2) - std::max(upperLeftY1, upperLeftY2));

  float volumei = (lowerRightX1 - upperLeftX1) * (lowerRightY1 - upperLeftY1);
  float volumej = (lowerRightX2 - upperLeftX2) * (lowerRightY2 - upperLeftY2);

  float unite = volumei + volumej - intersection;

  return intersection / unite;
}

float YoloRobotDetector::colorDistance(unsigned char& r1, unsigned char& r2, unsigned char& g1, unsigned char& g2, unsigned char& b1, unsigned char& b2)
{
  // See: https://www.compuphase.com/cmetric.htm
  int rmean = ((int)r1 + (int)r2) / 2;
  int r = (int)r1 - (int)r2;
  int g = (int)g1 - (int)g2;
  int b = (int)b1 - (int)b2;
  return std::sqrt(static_cast<float>((((512 + rmean)*r*r) >> 8) + 4 * g*g + (((767 - rmean)*b*b) >> 8)));
}

RobotEstimate::RobotType YoloRobotDetector::scanForRobotColor(
  const Vector2f &from,
  const Vector2f &scanDir,
  const int &maxSteps,
  const bool &upper)
{
  const Image &image = upper ? (Image&)theImageUpper : theImage;
  const FieldColors &fieldColor = upper ? (FieldColorsUpper&)theFieldColorsUpper : theFieldColors;

  int numOwnColor = 0;
  int numOppColor = 0;
  Vector2f checkPoint(from);
  int steps = 0;

  while (
    !image.isOutOfImage(checkPoint.x(), checkPoint.y(), 4) 
    && steps < maxSteps
    && (numOwnColor < 0.5* maxSteps || numOppColor < 0.5* maxSteps)
    )
  {
    Image::Pixel p = image[(int)checkPoint.y()][(int)checkPoint.x()];
    if (!fieldColor.isPixelFieldColor(p.y, p.cb, p.cr))
    {
      unsigned char ownColorR, ownColorG, ownColorB;
      ColorModelConversions::fromYCbCrToRGB(ownColor.y, ownColor.cb, ownColor.cr, ownColorR, ownColorG, ownColorB);

      unsigned char oppColorR, oppColorG, oppColorB;
      ColorModelConversions::fromYCbCrToRGB(oppColor.y, oppColor.cb, oppColor.cr, oppColorR, oppColorG, oppColorB);

      float diffBetweenColors = colorDistance(ownColorR, ownColorG, ownColorB, oppColorR, oppColorG, oppColorB);
      float diffToOwnColor = colorDistance(p.r, p.g, p.b, ownColorR, ownColorG, ownColorB);
      float diffToOwnColorNorm = diffToOwnColor / diffBetweenColors;
      float diffToOppColor = colorDistance(p.r, p.g, p.b, oppColorR, oppColorG, oppColorB);
      float diffToOppColorNorm = diffToOppColor / diffBetweenColors;

      // TODO fix for white team color!
      unsigned char whiteColorR = 255, whiteColorG = 128, whiteColorB = 255;
      float diffToWhite = colorDistance(p.r, p.g, p.b, whiteColorR, whiteColorG, whiteColorB);

      if (diffToWhite > 300) {
        if (diffToOwnColorNorm < diffToOppColorNorm)
          numOwnColor += (diffToOwnColorNorm < 1.3f);
        else
          numOppColor += (diffToOppColorNorm < 1.3f);
      }
      
      /*
      int dy = int(p.y) - ownColor.x();
      dy /= 2;
      int dcb = int(p.cb) - ownColor.y();
      int dcr = int(p.cr) - ownColor.z();
      float diffToOwnColor = std::sqrt(float(dy*dy + dcb*dcb + dcr*dcr));

      dy = int(p.y) - oppColor.x();
      dy /= 2;
      dcb = int(p.cb) - oppColor.y();
      dcr = int(p.cr) - oppColor.z();
      float diffToOppColor = std::sqrt(float(dy*dy + dcb*dcb + dcr*dcr));

      if (diffToOwnColor > 15.f || diffToOppColor > 15.f)
      {
        if (diffToOwnColor < diffToOppColor)
          numOwnColor += (diffToOwnColor < 130.f);
        else
          numOppColor += (diffToOppColor < 130.f);

      }
      */
    }
    checkPoint += scanDir;
    steps++;
  }

  if (upper) 
    LINE("module:YoloRobotDetector:shirtScanUpper", from.x(), from.y(), checkPoint.x(), checkPoint.y(), 3, Drawings::solidPen, ColorRGBA::black);
  else 
    LINE("module:YoloRobotDetector:shirtScan", from.x(), from.y(), checkPoint.x(), checkPoint.y(), 3, Drawings::solidPen, ColorRGBA::black);
  
  if (numOwnColor > 5 || numOppColor > 5) 
  {
    if (numOwnColor > static_cast<int>(std::floor((numOwnColor + numOppColor)*0.66f)) && numOppColor < static_cast<int>(std::floor((numOwnColor + numOppColor)*0.33f)))
      return RobotEstimate::teammateRobot;
    else if (numOppColor > static_cast<int>(std::floor((numOwnColor + numOppColor)*0.66f)) && numOwnColor < static_cast<int>(std::floor((numOwnColor + numOppColor)*0.33f)))
      return RobotEstimate::opponentRobot;
    else
      return RobotEstimate::unknownRobot;
  }
  else
    return RobotEstimate::unknownRobot;
}

void YoloRobotDetector::addObstacleFromBumpers()
{
  // Security check for broken bumpers
  if (theKeySymbols.obstacle_hit)
  {
    RobotEstimate robot;
    robot.fromUpperImage = false; // should cover the whole lower image
    robot.distance = 100; // directly in front of our feet
    robot.locationOnField.translation.x() = 100; // dito
    if (theKeySymbols.timeLeftFootPressed < theKeySymbols.timeRightFootPressed)
      robot.locationOnField.translation.y() = 50; // leftFootPressed
    else 
      robot.locationOnField.translation.y() = -50; // rightFootPressed
    robot.locationOnField.rotation = 0; // don't know...
    robot.robotType = RobotEstimate::unknownRobot;
    robot.validity = 0.85f;
    calcImageCoords(robot, false);
    localRobotsPercept.robots.push_back(robot);
  }
}

void YoloRobotDetector::calcImageCoords(RobotEstimate& robot, bool upper)
{
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  float heightInImage = Geometry::getSizeByDistance(cameraInfo, 550, robot.distance);
  Vector2f imageBase;
  bool ret = Transformation::robotToImage(robot.locationOnField.translation,
    cameraMatrix, cameraInfo, imageBase);

  if (ret == false)
  {
    robot.imageLowerRight.y() = 0;
    robot.imageLowerRight.x() = 0;
    robot.imageUpperLeft.x() = 0;
    robot.imageUpperLeft.y() = 0;
    return;
  }

  robot.imageLowerRight.y() = static_cast<int>(imageBase.y());
  robot.imageLowerRight.x() = static_cast<int>(imageBase.x()) + (int)(heightInImage / 5.f);
  robot.imageUpperLeft.x() = static_cast<int>(robot.imageLowerRight.x() - heightInImage / 2.5f);
  robot.imageUpperLeft.y() = static_cast<int>(robot.imageLowerRight.y() - heightInImage);

}

MAKE_MODULE(YoloRobotDetector, perception);
