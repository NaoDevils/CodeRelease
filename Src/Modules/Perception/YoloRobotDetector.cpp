#include "YoloRobotDetector.h"
#include "Tools/ImageProcessing/stb_image_write.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Transformation.h"
#include "Tools/ColorModelConversions.h"

#include "Tools/Math/sse_mathfun.h"

#include "Tools/Settings.h"
#include <taskflow/taskflow.hpp>

#include "Platform/File.h"

#include "Modules/Perception/CNNs/YoloRobotDetectorCNNs.h"

YoloRobotDetector::YoloRobotDetector()
{
  timeStamp = 0;
  timeStampUpper = 0;

  yoloParameterUpper.input_height = YoloRobotDetectorCNNUpper::input_height;
  yoloParameterUpper.input_width = YoloRobotDetectorCNNUpper::input_width;
  yoloParameterUpper.input_channel = YoloRobotDetectorCNNUpper::input_channel;
  yoloParameterUpper.output_height = YoloRobotDetectorCNNUpper::output_height;
  yoloParameterUpper.output_width = YoloRobotDetectorCNNUpper::output_width;
  yoloParameterUpper.num_of_boxes = YoloRobotDetectorCNNUpper::num_of_boxes;
  yoloParameterUpper.num_of_classes = YoloRobotDetectorCNNUpper::num_of_classes;
  yoloParameterUpper.num_of_coords = YoloRobotDetectorCNNUpper::num_of_coords;
  yoloParameterUpper.anchors = YoloRobotDetectorCNNUpper::anchors;

  detectionVectorUpper.reserve(yoloParameterUpper.output_height * yoloParameterUpper.output_width * yoloParameterUpper.num_of_boxes);
  inputVectorUpper.resize(yoloParameterUpper.input_height * yoloParameterUpper.input_width * yoloParameterUpper.input_channel, 0.f);
  localRobotsPerceptYoloUpper.robots.clear();

  yoloParameter.input_height = YoloRobotDetectorCNNLower::input_height;
  yoloParameter.input_width = YoloRobotDetectorCNNLower::input_width;
  yoloParameter.input_channel = YoloRobotDetectorCNNLower::input_channel;
  yoloParameter.output_height = YoloRobotDetectorCNNLower::output_height;
  yoloParameter.output_width = YoloRobotDetectorCNNLower::output_width;
  yoloParameter.num_of_boxes = YoloRobotDetectorCNNLower::num_of_boxes;
  yoloParameter.num_of_classes = YoloRobotDetectorCNNLower::num_of_classes;
  yoloParameter.num_of_coords = YoloRobotDetectorCNNLower::num_of_coords;
  yoloParameter.anchors = YoloRobotDetectorCNNLower::anchors;

  detectionVector.reserve(yoloParameter.output_height * yoloParameter.output_width * yoloParameter.num_of_boxes);
  inputVector.resize(yoloParameter.input_height * yoloParameter.input_width * yoloParameter.input_channel, 0.f);
  localRobotsPerceptYolo.robots.clear();

#ifdef NO_HORIZON
  yIdxs.reserve(yoloParameterUpper.input_height + 1);
#endif

  std::string filename = std::string(File::getBHDir()) + "/Config/nao_U16_V32_stride_res_no_horizon_bs032_ts001-8406.tflite";

  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());
  TFLITE_MINIMAL_CHECK(model != nullptr);

  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  TFLITE_MINIMAL_CHECK(interpreter != nullptr);

  interpreter->SetNumThreads(1);

  // Resize input tensors
  input_tensor = interpreter->inputs()[0];
  output_tensor = interpreter->outputs()[0];
  TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
  //int input_batch = input_dims->data[0];
  int input_height = input_dims->data[1];
  int input_width = input_dims->data[2];
  int input_channels = input_dims->data[3];

  std::vector<int> new_input;
  new_input.push_back(1);
  new_input.push_back(input_height);
  new_input.push_back(input_width);
  new_input.push_back(input_channels);
  interpreter->ResizeInputTensor(input_tensor, new_input);

  // Allocate memory fore the tensors
  interpreter->AllocateTensors();

  // Check interpreter state
  //tflite::PrintInterpreterState(interpreter.get());
}

void YoloRobotDetector::reset(const bool& upper)
{
  if (upper)
  {
    localRobotsPerceptYoloUpper.robots.clear();
    localBallHypotheses.ballSpotsUpper.clear();
    localPenaltyCrossHypotheses.penaltyCrossesUpper.clear();
  }
  else
  {
    localRobotsPerceptYolo.robots.clear();
    localBallHypotheses.ballSpots.clear();
    localPenaltyCrossHypotheses.penaltyCrosses.clear();
  }
}

void YoloRobotDetector::execute(tf::Subflow& subflow)
{
  subflow
      .emplace(
          [=]()
          {
            execute(true);
          })
      .priority(tf::TaskPriority::HIGH)
      .name("YoloUpper [YoloRobotDetector]");

  subflow
      .emplace(
          [=]()
          {
            execute(false);
          })
      .priority(tf::TaskPriority::LOW)
      .name("YoloLower [YoloRobotDetector]");
}

void YoloRobotDetector::update(RobotsHypothesesYolo& theRobotsHypothesesYolo)
{
  theRobotsHypothesesYolo.robots.clear();
  addObstacleFromBumpers();

  theRobotsHypothesesYolo = localRobotsPerceptYolo;
}

void YoloRobotDetector::update(RobotsHypothesesYoloUpper& theRobotsHypothesesYoloUpper)
{
  theRobotsHypothesesYoloUpper.robots.clear();

  theRobotsHypothesesYoloUpper = localRobotsPerceptYoloUpper;
}

void YoloRobotDetector::update(BallHypothesesYolo& theBallHypothesesYolo)
{
  theBallHypothesesYolo.ballSpots.clear();
  theBallHypothesesYolo.ballSpotsUpper.clear();

  bool sortBallSpots = true;

  if (sortBallSpots)
  {
    auto sorting_criteria = [](const auto& a, const auto& b)
    {
      //return a.validity > b.validity;
      return a.position.y() > b.position.y();
    };
    std::sort(localBallHypotheses.ballSpots.begin(), localBallHypotheses.ballSpots.end(), sorting_criteria);
    std::sort(localBallHypotheses.ballSpotsUpper.begin(), localBallHypotheses.ballSpotsUpper.end(), sorting_criteria);
  }

  theBallHypothesesYolo = localBallHypotheses;
}

void YoloRobotDetector::update(PrePenaltyCrossHypothesesYolo& thePrePenaltyCrossHypothesesYolo)
{
  thePrePenaltyCrossHypothesesYolo.penaltyCrosses.clear();
  thePrePenaltyCrossHypothesesYolo.penaltyCrossesUpper.clear();

  std::sort(localPenaltyCrossHypotheses.penaltyCrosses.begin(), localPenaltyCrossHypotheses.penaltyCrosses.end());
  std::sort(localPenaltyCrossHypotheses.penaltyCrossesUpper.begin(), localPenaltyCrossHypotheses.penaltyCrossesUpper.end());

  thePrePenaltyCrossHypothesesYolo = localPenaltyCrossHypotheses;
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
  const Image& image = upper ? (Image&)theImageUpper : theImage;
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

    if (!image.shouldBeProcessed())
      return;

    const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
    if (theFallDownState.state != FallDownState::upright || cameraMatrix.isValid == false)
      return;

    RobotsPercept& localPercepts = upper ? (RobotsPercept&)localRobotsPerceptYoloUpper : localRobotsPerceptYolo;
    YoloParameter& localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;
    const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
    std::vector<float>& input = upper ? (std::vector<float>&)inputVectorUpper : inputVector;
    std::vector<YoloDetection>& localDetectionVector = upper ? (std::vector<YoloDetection>&)detectionVectorUpper : detectionVector;
    localDetectionVector.clear();

    YoloResult result(localParameter.output_height, localParameter.output_width, localParameter.num_of_boxes, localParameter.num_of_coords, localParameter.num_of_classes);

    STOPWATCH(upper ? "YOLO-CopyUpper" : "YOLO-CopyLower")
    {
      if (localParameter.input_channel == 3)
      {
        if (!upper)
        {
          image.copyAndResizeArea<true, false>({0, 0}, {image.width, image.height}, {localParameter.input_width, localParameter.input_height}, input.data());
        }
        else
        {
          if (!useTFlite)
          {
#ifdef NO_HORIZON
            int minY = 0;
            const Geometry::Line horizon = Geometry::calculateHorizon(cameraMatrix, cameraInfo);
            if (!image.isOutOfImage(horizon.base.x(), horizon.base.y(), 4))
              minY = std::min(std::max(0, static_cast<int>(horizon.base.y()) + 4), static_cast<int>(image.height - localParameter.input_height));
            yIdxs = image.copyAndResizeRGBFloatNoHorizon(localParameter.input_width, localParameter.input_height, minY, &input[0]);
#else
            image.copyAndResizeArea<true, false>({0, 0}, {image.width, image.height}, {localParameter.input_width, localParameter.input_height}, input.data());
#endif
          }
        }
      }
      else
      {
        image.copyAndResizeArea<false, false>({0, 0}, {image.width, image.height}, {localParameter.input_width, localParameter.input_height}, input.data());
      }
    }

    if (upper && useTFlite)
    {
      STOPWATCH("YOLO-CopyUpper_tflite")
      {
#ifdef NO_HORIZON
        int minY = 0;
        const Geometry::Line horizon = Geometry::calculateHorizon(cameraMatrix, cameraInfo);
        if (!image.isOutOfImage(horizon.base.x(), horizon.base.y(), 4))
          minY = std::min(std::max(0, static_cast<int>(horizon.base.y()) + 4), static_cast<int>(image.height - localParameter.input_height));
        yIdxs = image.copyAndResizeRGBFloatNoHorizon(localParameter.input_width, localParameter.input_height, minY, interpreter->typed_tensor<float>(input_tensor));
#else
        image.copyAndResizeArea<true, false>({0, 0}, {image.width, image.height}, {localParameter.input_width, localParameter.input_height}, interpreter->typed_tensor<float>(input_tensor));
#endif
      }
    }

    if (upper)
    {
      COMPLEX_IMAGE(YoloDebugImageUpper)
      {
        INIT_DEBUG_IMAGE_BLACK(YoloDebugImageUpper, image.width, image.height);
        int factor = image.width / yoloParameterUpper.input_width;

        for (int i = 0; i < image.height; i++)
        {
          for (int j = 0; j < image.width; j++)
          {
            int i_scaled = i / factor;
            int j_scaled = j / factor;
            if (yoloParameterUpper.input_channel == 3)
            {
              float pixely = input[i_scaled * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j_scaled * yoloParameterUpper.input_channel + 0] * 255.f;
              float pixelu = input[i_scaled * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j_scaled * yoloParameterUpper.input_channel + 1] * 255.f;
              float pixelv = input[i_scaled * yoloParameterUpper.input_width * yoloParameterUpper.input_channel + j_scaled * yoloParameterUpper.input_channel + 2] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_RGB(YoloDebugImageUpper, j, i, static_cast<unsigned char>(pixely), static_cast<unsigned char>(pixelu), static_cast<unsigned char>(pixelv));
            }
            else
            {
              float pixel = input[i_scaled * yoloParameterUpper.input_width + j_scaled] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_YUV(YoloDebugImageUpper, j, i, static_cast<unsigned char>(pixel), 127, 127);
            }
          }
        }
        SEND_DEBUG_IMAGE(YoloDebugImageUpper);
      }
    }
    else
    {
      COMPLEX_IMAGE(YoloDebugImage)
      {
        INIT_DEBUG_IMAGE_BLACK(YoloDebugImage, image.width, image.height);
        int factor = image.width / yoloParameter.input_width;
        for (int i = 0; i < image.height; i++)
        {
          for (int j = 0; j < image.width; j++)
          {
            int i_scaled = i / factor;
            int j_scaled = j / factor;
            if (yoloParameter.input_channel == 3)
            {
              float pixely = input[i_scaled * yoloParameter.input_width * yoloParameter.input_channel + j_scaled * yoloParameter.input_channel + 0] * 255.f;
              float pixelu = input[i_scaled * yoloParameter.input_width * yoloParameter.input_channel + j_scaled * yoloParameter.input_channel + 1] * 255.f;
              float pixelv = input[i_scaled * yoloParameter.input_width * yoloParameter.input_channel + j_scaled * yoloParameter.input_channel + 2] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_RGB(YoloDebugImage, j, i, static_cast<unsigned char>(pixely), static_cast<unsigned char>(pixelu), static_cast<unsigned char>(pixelv));
            }
            else
            {
              float pixel = input[i_scaled * yoloParameter.input_width + j_scaled] * 255.f;
              DEBUG_IMAGE_SET_PIXEL_YUV(YoloDebugImage, j, i, static_cast<unsigned char>(pixel), 127, 127);
            }
          }
        }
        SEND_DEBUG_IMAGE(YoloDebugImage);
      }
    }

    if (upper)
    {
      if (!useTFlite)
      {
        STOPWATCH("YOLO-ExecutionUpper")
        {
          YoloRobotDetectorCNNUpper::cnn(input.data(), result.result.data());
        }
      }
      else
      {
        STOPWATCH("YOLO-ExecutionUpper-tflite")
        {
          float* output = interpreter->typed_tensor<float>(output_tensor);

          if (interpreter->Invoke() != kTfLiteOk)
          {
            OUTPUT_ERROR("Failed to invoke tflite!");
          }

          std::copy(output, output + result.result.size(), result.result.data());
        }
      }
    }
    else
    {
      STOPWATCH("YOLO-Execution")
      {
        YoloRobotDetectorCNNLower::cnn(input.data(), result.result.data());
      }
    }

    STOPWATCH(upper ? "YOLO-PostprocessingUpper" : "YOLO-PostprocessingLower")
    {
      generateNetworkBoxes(1, localDetectionVector, result, upper);
      std::sort(localDetectionVector.begin(), localDetectionVector.end());

      float nms = upper ? upperNMSThreshold : lowerNMSThreshold;
      if (nms < 1.0f)
      {
        for (size_t i = 0; i < localDetectionVector.size(); i++)
        {
          if (localDetectionVector[i].prob == 0.f)
          {
            continue;
          }
          else
          {
            for (size_t j = i + 1; j < localDetectionVector.size(); j++)
            {
              if (iou(localDetectionVector[i].bbox, localDetectionVector[j].bbox, image.height, image.width) >= nms)
              {
                localDetectionVector[j].prob = 0.f;
              }
            }
          }
        }
      }

      for (size_t boxNo = 0; boxNo < localDetectionVector.size(); boxNo++)
      {
        const YoloDetection& det = localDetectionVector[boxNo];
        bool localUseYoloHeight = upper ? useUpperYoloHeight : useLowerYoloHeight;
        if (det.prob > 0.f)
        {
          float x = det.bbox.x * image.width;
          float w = det.bbox.w * image.width;
          float y = det.bbox.y * image.height;
          float h = det.bbox.h * image.height;

#ifdef NO_HORIZON
          if (upper)
          {
            y = det.bbox.y * localParameter.input_height;
            h = det.bbox.h * localParameter.input_height;
            int yMin = std::max(0, static_cast<int>((y - (h / 2.f)) + 0.5));
            int yMax = std::min(static_cast<int>((y + (h / 2.f)) + 0.5), static_cast<int>(yoloParameterUpper.input_height));
            int newYMin = yIdxs[yMin];
            int newYMax = yIdxs[yMax];
            h = static_cast<float>(newYMax - newYMin);
            y = static_cast<float>(newYMin + (h / 2.f));
          }
#endif

          if (useXOffset)
          {
            // yolo percepts are not centered correctly
            // the more near the percept is to the image corner,
            // the more of the x-offset needs to be used!
            float imgWidthHalf = image.width / 2.f;
            float sign = (x < imgWidthHalf ? 1.f : -1.f);
            float usedXOffset = (upper ? xOffset : xOffset / 2.f);
            x += sign * usedXOffset * (1 - abs(std::min(x, image.width - x) / imgWidthHalf));
          }

          if (det.sortClass == YoloClasses::Robot)
          {
            //// ROBOT ////
            RobotEstimate re;
            re.locationOnField.rotation = 0;
            re.fromUpperImage = upper;
            re.validity = det.prob;
            re.timestampFromImage = image.timeStamp;

            Vector2f boxCenter(x, y);
            Vector2f vecBoxCenterToUpperLeft(-(w) / 2.f, -(h) / 2.f);
            re.imageUpperLeft = (boxCenter + vecBoxCenterToUpperLeft).cast<int>();
            re.imageLowerRight = (boxCenter - vecBoxCenterToUpperLeft).cast<int>();

            if (Transformation::imageToRobot(Vector2f(boxCenter.x(), re.imageLowerRight.y()), cameraMatrix, cameraInfo, re.locationOnField.translation)
                && !(re.locationOnField.translation.x() < 100 && re.locationOnField.translation.y() < 2000) && re.locationOnField.translation.x() > -1000)
            {

              re.distance = re.locationOnField.translation.norm();
              float heightInImage = 0.f;

              if (!localUseYoloHeight)
              {
                heightInImage = Geometry::getSizeByDistance(cameraInfo, 580.f, re.distance);
                float widthMismatchFactor = w / (heightInImage * (2.f / 5.f));
                if (widthMismatchFactor > 1)
                  widthMismatchFactor = 1 / widthMismatchFactor;
                re.validity *= widthMismatchFactor;
                if ((upper && re.validity < upperRobotThreshold) || (!upper && re.validity < lowerRobotThreshold))
                  continue;
              }
              else
              {
                heightInImage = h;
              }
              re.imageLowerRight.y() = static_cast<int>(re.imageLowerRight.y());
              re.imageLowerRight.x() = static_cast<int>(boxCenter.x() + heightInImage / 5.f);
              re.imageUpperLeft.x() = static_cast<int>(boxCenter.x() - heightInImage / 5.f);
              re.imageUpperLeft.y() = static_cast<int>(re.imageLowerRight.y() - heightInImage);

              localPercepts.robots.push_back(re);
            }
          }
          else if (det.sortClass == YoloClasses::Ball)
          {
            //// BALL ////
            std::vector<BallSpot> bsy;

            BallSpot bs;
            bs.position = Vector2f(x, y).cast<int>();
            bs.radiusInImage = std::min<float>(w / 2.f, h / 2.f);
            bs.validity = det.prob;
            bs.upper = upper;
            bsy.push_back(bs);

            // theoretical diameter if cameramatrix is correct
            BallSpot bsh(bs);
            Vector2f posOnField;
            Geometry::Circle expectedCircle;
            if (Transformation::imageToRobotHorizontalPlane(bsh.position.cast<float>(), theFieldDimensions.ballRadius, cameraMatrix, cameraInfo, posOnField)
                && Geometry::calculateBallInImage(posOnField, cameraMatrix, cameraInfo, theFieldDimensions.ballRadius, expectedCircle)
                && (expectedCircle.radius > (bs.radiusInImage * 1.2f) || expectedCircle.radius < (bs.radiusInImage * (1 / 1.2f))))
            {
              bsh.radiusInImage = expectedCircle.radius;
              bsh.validity = det.prob + 0.01f;
              bsy.push_back(bsh);
            }

            if (upper)
            {
              for (std::vector<BallSpot>::const_iterator i = bsy.begin(); i != bsy.end(); ++i)
              {
                localBallHypotheses.ballSpotsUpper.push_back(*i);
              }
            }
            else
            {
              for (std::vector<BallSpot>::const_iterator i = bsy.begin(); i != bsy.end(); ++i)
              {
                localBallHypotheses.ballSpots.push_back(*i);
              }
            }
          }
          else if (det.sortClass == YoloClasses::Penaltycross)
          {
            //// PENALTY CROSS ////
            PenaltyCross pc;
            pc.positionInImage = Vector2f(x, y);
            pc.validity = det.prob;
            if (upper)
            {
              localPenaltyCrossHypotheses.penaltyCrossesUpper.push_back(pc);
            }
            else
            {
              localPenaltyCrossHypotheses.penaltyCrosses.push_back(pc);
            }
          }
        }
      }
    }
  }
}

void YoloRobotDetector::generateNetworkBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, YoloResult& yoloResult, const bool& upper)
{
  YoloParameter& localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;

  for (unsigned int i = 0; i < localParameter.output_width * localParameter.output_height; ++i)
  {
    int row = i / localParameter.output_width;
    int col = i % localParameter.output_width;
    for (unsigned int n = 0; n < localParameter.num_of_boxes; ++n)
    {
      YoloDetection newDetection;
      YoloRegionBox newBox;
      getRegionBox(newBox, yoloResult, n, col, row, localParameter);

      newDetection.bbox = newBox;

      float conf = newBox.conf; // object confidence calculated in getRegionBox(..)

      // calculate soft max over class probability values (they represent a prob. distribution) to find the most probable class
      float maxScore = -INFINITY;
      int bestClassIndex = 0;

      if (localParameter.num_of_classes > 1)
      {
        std::vector<float> classPredictions;
        int baseIndex = n * (localParameter.num_of_coords + 1 + localParameter.num_of_classes) + localParameter.num_of_coords + 1;
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
      else
      { // Special Conditions - only one class
        maxScore = 1;
        bestClassIndex = 0;
      }

      float max_class_conf = conf * maxScore;
      if ((bestClassIndex == YoloClasses::Robot && max_class_conf > (upper ? upperRobotThreshold : lowerRobotThreshold))
          || (bestClassIndex == YoloClasses::Ball && max_class_conf > (upper ? upperBallThreshold : lowerBallThreshold))
          || (bestClassIndex == YoloClasses::Penaltycross && max_class_conf > (upper ? upperPenaltyCrossThreshold : lowerPenaltyCrossThreshold)))
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

void YoloRobotDetector::getRegionBox(YoloRegionBox& b, YoloResult& yoloResult, int numOfBox, int col, int row, const YoloParameter& localParameter)
{
  int baseIndex = numOfBox * (localParameter.num_of_coords + 1 + localParameter.num_of_classes);
  alignas(16) float input[4] = {-yoloResult(row, col, baseIndex), -yoloResult(row, col, baseIndex + 1), yoloResult(row, col, baseIndex + 2), yoloResult(row, col, baseIndex + 3)};
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

void YoloRobotDetector::correctRegionBoxes(int relative, std::vector<YoloDetection>& localDetectionVector, const bool& upper)
{
  YoloParameter& localParameter = upper ? (YoloParameter&)yoloParameterUpper : yoloParameter;

  int new_w = 0;
  int new_h = 0;
  if (((float)localParameter.input_width / localParameter.output_width) < ((float)localParameter.input_height / localParameter.output_height))
  {
    new_w = localParameter.input_width;
    new_h = (localParameter.output_height * localParameter.input_width) / localParameter.output_width;
  }
  else
  {
    new_h = localParameter.input_height;
    new_w = (localParameter.output_width * localParameter.input_height) / localParameter.output_height;
  }

  for (std::vector<YoloDetection>::iterator it = localDetectionVector.begin(); it != localDetectionVector.end(); ++it)
  {
    (*it).bbox.w *= (float)localParameter.input_width / new_w;
    (*it).bbox.h *= (float)localParameter.input_height / new_h;
    (*it).bbox.x = ((*it).bbox.x - (float)((localParameter.input_width - new_w) / 2.f / localParameter.input_width)) / ((float)new_w / localParameter.input_width);
#ifdef BOTTOM_AS_CENTER
    (*it).bbox.y = ((*it).bbox.y - ((*it).bbox.h / 2.f) - (float)((localParameter.input_height - new_h) / 2. / localParameter.input_height)) / ((float)new_h / localParameter.input_height);
#else
    (*it).bbox.y = ((*it).bbox.y - (float)((localParameter.input_height - new_h) / 2.f / localParameter.input_height)) / ((float)new_h / localParameter.input_height);
#endif
  }
}

float YoloRobotDetector::iou(YoloRegionBox& box1, YoloRegionBox& box2, int heigth, int width)
{
  float lowerRightX1 = box1.x * width + (box1.w * width / 2.f);
  float lowerRightX2 = box2.x * width + (box2.w * width / 2.f);
  float upperLeftX1 = box1.x * width - (box1.w * width / 2.f);
  float upperLeftX2 = box2.x * width - (box2.w * width / 2.f);

  float lowerRightY1 = box1.y * heigth + (box1.h * heigth / 2.f);
  float lowerRightY2 = box2.y * heigth + (box2.h * heigth / 2.f);
  float upperLeftY1 = box1.y * heigth - (box1.h * heigth / 2.f);
  float upperLeftY2 = box2.y * heigth - (box2.h * heigth / 2.f);

  float intersection = std::max(0.f, std::min(lowerRightX1, lowerRightX2) - std::max(upperLeftX1, upperLeftX2))
      * std::max(0.f, std::min(lowerRightY1, lowerRightY2) - std::max(upperLeftY1, upperLeftY2));

  float volumei = (lowerRightX1 - upperLeftX1) * (lowerRightY1 - upperLeftY1);
  float volumej = (lowerRightX2 - upperLeftX2) * (lowerRightY2 - upperLeftY2);

  float unite = volumei + volumej - intersection;

  return intersection / unite;
}

void YoloRobotDetector::addObstacleFromBumpers()
{
  // Security check for broken bumpers and upright
  if (theKeySymbols.obstacle_hit && theFallDownState.state == FallDownState::upright)
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
    localRobotsPerceptYolo.robots.push_back(robot);
  }
}

void YoloRobotDetector::calcImageCoords(RobotEstimate& robot, bool upper)
{
  const CameraInfo& cameraInfo = upper ? (CameraInfo&)theCameraInfoUpper : theCameraInfo;
  const CameraMatrix& cameraMatrix = upper ? (CameraMatrix&)theCameraMatrixUpper : theCameraMatrix;
  float heightInImage = Geometry::getSizeByDistance(cameraInfo, 550, robot.distance);
  Vector2f imageBase;
  bool ret = Transformation::robotToImage(robot.locationOnField.translation, cameraMatrix, cameraInfo, imageBase);

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
