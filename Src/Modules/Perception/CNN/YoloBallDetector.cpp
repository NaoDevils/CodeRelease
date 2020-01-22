#include "YoloBallDetector.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ColorModelConversions.h"

#include "Tools/Math/sse_mathfun.h"

#include "Tools/Settings.h"

#ifdef WINDOWS
#include "direct.h"
#endif

#define LAYER_STOPWATCH

////// UPPER V6 /////
namespace upperBallDetector {
  #include "sep-1573-dky.c"
}

/////// LOWER V6 /////
namespace lowerBallDetector {
  #include "sep-1573-dky.c"
}


YoloBallDetector::YoloBallDetector()
{
  timeStamp = 0;
  timeStampUpper = 0;

  yoloParameterUpper.input_height = upperBallDetector::input_height;
  yoloParameterUpper.input_width = upperBallDetector::input_width;
  yoloParameterUpper.input_channel = upperBallDetector::input_channel;
  yoloParameterUpper.output_height = upperBallDetector::output_height;
  yoloParameterUpper.output_width = upperBallDetector::output_width;
  yoloParameterUpper.num_of_boxes = upperBallDetector::num_of_boxes;
  yoloParameterUpper.num_of_classes = upperBallDetector::num_of_classes;
  yoloParameterUpper.num_of_coords = upperBallDetector::num_of_coords;
  yoloParameterUpper.anchors = upperBallDetector::anchors;

  detectionVectorUpper.reserve(yoloParameterUpper.output_height * yoloParameterUpper.output_width * yoloParameterUpper.num_of_boxes);
  inputVectorUpper.resize(yoloParameterUpper.input_height * yoloParameterUpper.input_width * yoloParameterUpper.input_channel, 0.f);

  yoloParameter.input_height = lowerBallDetector::input_height;
  yoloParameter.input_width = lowerBallDetector::input_width;
  yoloParameter.input_channel = lowerBallDetector::input_channel;
  yoloParameter.output_height = lowerBallDetector::output_height;
  yoloParameter.output_width = lowerBallDetector::output_width;
  yoloParameter.num_of_boxes = lowerBallDetector::num_of_boxes;
  yoloParameter.num_of_classes = lowerBallDetector::num_of_classes;
  yoloParameter.num_of_coords = lowerBallDetector::num_of_coords;
  yoloParameter.anchors = lowerBallDetector::anchors;

  inputVector.resize(yoloParameter.input_height * yoloParameter.input_width * yoloParameter.input_channel, 0.f);
}

void YoloBallDetector::reset(const bool& upper) {
  if (upper)
  {
    localBallHypothesesUpper.ballSpotsUpper.clear();
  }
  else
  {
    localBallHypothesesUpper.ballSpots.clear();
  }
}

void YoloBallDetector::update(BallHypothesesYoloUpper& theBallHypothesesYoloUpper) {
  theBallHypothesesYoloUpper.ballSpots.clear();

  execute(true);
  // execute(false);

  theBallHypothesesYoloUpper = localBallHypothesesUpper;
}

void YoloBallDetector::update(YoloInputUpper& theYoloInputUpper)
{
  theYoloInputUpper.image = inputVectorUpper;
  theYoloInputUpper.width = yoloParameterUpper.input_width;
  theYoloInputUpper.height = yoloParameterUpper.input_height;
  theYoloInputUpper.channel = yoloParameterUpper.input_channel;
  theYoloInputUpper.timeStamp = theImageUpper.timeStamp;
  theYoloInputUpper.imageUpdated = true;
}


void YoloBallDetector::execute(const bool& upper)
{
  DECLARE_DEBUG_DRAWING("module:YoloBallDetector:croppedImageUpper", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:YoloBallDetector:croppedImageLower", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:YoloBallDetector:ballPosition", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:YoloBallDetector:cropBox", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:YoloBallDetector:noOfBoundingBoxes", "drawingOnImage");


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

    YoloParameter &localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;
    const CameraInfo &cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    std::vector<float> &input = upper ? (std::vector<float>&)inputVectorUpper : inputVector;
    std::vector<YoloDetection> &localDetectionVector = upper ? (std::vector<YoloDetection>&)detectionVectorUpper : detectionVector;
    localDetectionVector.clear();

    YoloResult result(localParameter.output_height, localParameter.output_width, localParameter.num_of_boxes, localParameter.num_of_coords, localParameter.num_of_classes);

    Vector2f pImage = Vector2f::Zero();
    if (!Transformation::robotToImage(theBallModel.estimate.position, cameraMatrix, cameraInfo, pImage))
      return;
    Vector2f pImageMinY = Vector2f(0.f, std::max(Geometry::calculateHorizon(cameraMatrix, cameraInfo).base.y(), 0.f));
    pImageMinY.y() = std::min(pImageMinY.y(), static_cast<float>(cameraInfo.height - localParameter.input_height - 1));

    pImage.x() = std::min<float>(std::max<float>(static_cast<float>(0 + localParameter.input_width / 2), pImage.x()),
      static_cast<float>(cameraInfo.width - localParameter.input_width / 2 - 1));
    int xStart = (int)pImage.x() - localParameter.input_width / 2;
    Geometry::Circle ballInImage;
    if (!Geometry::calculateBallInImage(theBallModel.estimate.position, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, ballInImage))
      return;
    int yOffsetDown = static_cast<int>(localParameter.input_height - 5 * ballInImage.radius / 2); // keep ball in yolo area on the upper end
    int yStart = std::max(static_cast<int>(pImageMinY.y()), std::min<int>(cameraInfo.height - localParameter.input_height - 1, (int)pImage.y() - (localParameter.input_height - yOffsetDown)));
    //yStart = std::min<int>(cameraInfo.height - localParameter.input_height, std::max(yStart, static_cast<int>(pImageMinY.y())));
    pImage.y() = static_cast<float>(yStart + localParameter.input_height / 2);
    
    if (upper){
      RECTANGLE("module:YoloBallDetector:croppedImageUpper", xStart, yStart, xStart+localParameter.input_width, yStart+localParameter.input_height, 3, Drawings::solidPen, ColorRGBA::red);
    }
    else {
      RECTANGLE("module:YoloBallDetector:croppedImageLower", xStart, yStart, xStart+localParameter.input_width, yStart+localParameter.input_height, 3, Drawings::solidPen, ColorRGBA::red);
    }
    // DRAWTEXT("module:YoloBallDetector:cropBox", 20, 200, 20, ColorRGBA(0, 255, 255), xStart);

      // std::stringstream ss;
      // ss << "Ball: " << pImage.x() << " xStart" << xStart;
      // OUTPUT_TEXT(ss.str());

    STOPWATCH("YOLO-Copy")
    {
      if (localParameter.input_channel == 3) {
        image.copyAndResizeAreaRGBFloat(xStart, yStart, localParameter.input_width, localParameter.input_height, localParameter.input_width, localParameter.input_height, &input[0]);
      }
      else {
        image.copyAndResizeAreaFloat(xStart, yStart, localParameter.input_width, localParameter.input_height, localParameter.input_width, localParameter.input_height, &input[0]);
      }
    }

    if (upper)
    {
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
        upperBallDetector::cnn(input.data(), result.result.data());
      }
    }
    else {
      STOPWATCH("YOLO-Execution")
      {
        lowerBallDetector::cnn(input.data(), result.result.data());
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
      if (upper) {
        DRAWTEXT("module:YoloBallDetector:noOfBoundingBoxes",
          image.width / 2, image.height / 2,
          15, ColorRGBA::yellow, "no. of boxes : " << localDetectionVector.size());
      }

      for (size_t boxNo = 0; boxNo < localDetectionVector.size(); boxNo++) {
        const YoloDetection &det = localDetectionVector[boxNo];

        if (det.prob > 0.f)
        {
          if (det.sortClass == YoloClasses::Ball)
          {
            //// BALL ////
            BallSpot bsy;
            bsy.position = Vector2f(det.bbox.x * localParameter.input_width + xStart, det.bbox.y * localParameter.input_height + yStart).cast<int>();
            bsy.radiusInImage = std::min<float>((det.bbox.w * localParameter.input_width) / 2, (det.bbox.h * localParameter.input_height) / 2);

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
            //##
            bsy.validity = det.prob;
            bsy.cb = 127;
            bsy.cr = 127;
            bsy.y = 0;
            // DRAWTEXT("module:YoloBallDetector:ballValidity",
            //   image.width / 2, image.height / 2,
            //   15, ColorRGBA::yellow, "validity : " << bsy.validity);
            if (upper)
            {
              localBallHypothesesUpper.ballSpotsUpper.push_back(bsy);
            }
            else
            {
              localBallHypothesesUpper.ballSpots.push_back(bsy);
            }
          }
          else {
            break; // there shouldn't be other classes (for now)
          }
        }
      }
    }
  }
}


void YoloBallDetector::generateNetworkBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, YoloResult& yoloResult, const bool &upper)
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
      int bestClassIndex = 1;

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
        bestClassIndex = 1;
      }

      float max_class_conf = conf*maxScore;
      if (bestClassIndex == YoloClasses::Ball && max_class_conf > (upper ? detectionThresholdUpperBall : detectionThresholdBall))
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

void YoloBallDetector::getRegionBox(YoloRegionBox &b,
  YoloResult& yoloResult, int numOfBox,
  int col, int row,
  const YoloParameter &localParameter
)
{
  alignas(16) float input[4] = { -yoloResult(row,col,0), -yoloResult(row,col,1), yoloResult(row,col,2), yoloResult(row,col,3) };
  alignas(16) float result[4];
  _mm_store_ps(result, exp_ps(_mm_load_ps(&input[0])));

  // all values get absolute by dividing through width (height) -> get image coordinates by multiplying with image width (height) later
  b.x = (col + 1 / (1 + result[0])) / localParameter.output_width; // paper: c_x + sigmoid t_x
  b.y = (row + 1 / (1 + result[1])) / localParameter.output_height; // paper: c_y + sigmoid t_y
  b.w = result[2] * localParameter.anchors[2 * numOfBox] / localParameter.output_width; // paper: p_w * e^(t_w) with p_w being anchor w
  b.h = result[3] * localParameter.anchors[2 * numOfBox + 1] / localParameter.output_height; // paper: p_h * e^(t_h) with p_h being anchor h
  b.conf = 1 / (1 + std::exp(-yoloResult(row, col, 4))); // paper: sigmoid of (t_o) with t_o being object confidence
  b.grid_x = col;
  b.grid_y = row;
}

void YoloBallDetector::correctRegionBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, const bool &upper)
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

float YoloBallDetector::iou(YoloRegionBox &box1, YoloRegionBox &box2, int heigth, int width) {
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

// float YoloBallDetector::colorDistance(unsigned char& r1, unsigned char& r2, unsigned char& g1, unsigned char& g2, unsigned char& b1, unsigned char& b2)
// {
//   // See: https://www.compuphase.com/cmetric.htm
//   int rmean = ((int)r1 + (int)r2) / 2;
//   int r = (int)r1 - (int)r2;
//   int g = (int)g1 - (int)g2;
//   int b = (int)b1 - (int)b2;
//   return std::sqrt(static_cast<float>((((512 + rmean)*r*r) >> 8) + 4 * g*g + (((767 - rmean)*b*b) >> 8)));
// }

MAKE_MODULE(YoloBallDetector, perception);
