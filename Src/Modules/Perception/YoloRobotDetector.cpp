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
  localRobotsPerceptUpper.robots.clear();

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
  localRobotsPercept.robots.clear();

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
  TfLiteIntArray* intput_dims = interpreter->tensor(input_tensor)->dims;
  //int intput_batch = intput_dims->data[0];
  int intput_height = intput_dims->data[1];
  int intput_width = intput_dims->data[2];
  int intput_channels = intput_dims->data[3];

  std::vector<int> new_input;
  new_input.push_back(1);
  new_input.push_back(intput_height);
  new_input.push_back(intput_width);
  new_input.push_back(intput_channels);
  interpreter->ResizeInputTensor(input_tensor, new_input);

  // Allocate memory fore the tensors
  interpreter->AllocateTensors();

  // Check interpreter state
  //tflite::PrintInterpreterState(interpreter.get());

  ownColor = ColorRGBA::yellow; // Default Yellow
  ownColor = ColorRGBA::black; // Default Black

  class_init();
}

void YoloRobotDetector::reset(const bool& upper)
{
  if (upper)
  {
    localRobotsPerceptUpper.robots.clear();
    localBallHypotheses.ballSpotsUpper.clear();
    localPenaltyCrossHypotheses.penaltyCrossesUpper.clear();
  }
  else
  {
    localRobotsPercept.robots.clear();
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
      .name("YoloUpper [YoloRobotDetector]");

  subflow
      .emplace(
          [=]()
          {
            execute(false);
          })
      .name("YoloLower [YoloRobotDetector]");
}

void YoloRobotDetector::update(RobotsPercept& theRobotsPercept)
{
  theRobotsPercept.robots.clear();
  setRobotColorFromGC();
  addObstacleFromBumpers();

  theRobotsPercept = localRobotsPercept;
}

void YoloRobotDetector::update(RobotsPerceptUpper& theRobotsPerceptUpper)
{
  DECLARE_DEBUG_DRAWING("module:YoloRobotDetector:shirtScanUpper", "drawingOnImage");
  theRobotsPerceptUpper.robots.clear();
  setRobotColorFromGC();

  theRobotsPerceptUpper = localRobotsPerceptUpper;
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

void YoloRobotDetector::update(PenaltyCrossHypothesesYolo& thePenaltyCrossHypothesesYolo)
{
  thePenaltyCrossHypothesesYolo.penaltyCrosses.clear();
  thePenaltyCrossHypothesesYolo.penaltyCrossesUpper.clear();

  thePenaltyCrossHypothesesYolo = localPenaltyCrossHypotheses;
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
  DECLARE_PLOT("module:YoloRobotDetector:robotPercepts");
  DECLARE_PLOT("module:YoloRobotDetector:validatedRobotPercepts");
  DECLARE_PLOT("module:YoloRobotDetector:declinedRobotPercepts");
  short robotPercepts = 0, validatedRobotPercepts = 0;
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

    RobotsPercept& localPercepts = upper ? (RobotsPercept&)localRobotsPerceptUpper : localRobotsPercept;
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
          image.copyAndResizeAreaRGBFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, &input[0]);
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
            image.copyAndResizeAreaRGBFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, &input[0]);
#endif
          }
        }
      }
      else
      {
        image.copyAndResizeAreaFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, &input[0]);
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
        image.copyAndResizeAreaRGBFloat(0, 0, image.width, image.height, localParameter.input_width, localParameter.input_height, interpreter->typed_tensor<float>(input_tensor));
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


          if (det.sortClass == YoloClasses::Robot)
          {
            //// ROBOT ////
            robotPercepts += 1;
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
                heightInImage = Geometry::getSizeByDistance(cameraInfo, 550.f, re.distance);
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
              STOPWATCH("YOLO-checkRobotEstimate")
              {
                if (re.fromUpperImage && class_checkEstimate(image, re))
                {
                  STOPWATCH("YOLO-updateRobotColor")
                  {
                    updateRobotColor(re);
                  }
                  validatedRobotPercepts += 1;
                  localPercepts.robots.push_back(re);
                }
              }
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
            bs.cb = 128;
            bs.cr = 128;
            bs.y = 250;
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
        PLOT("module:YoloRobotDetector:robotPercepts", robotPercepts);
        PLOT("module:YoloRobotDetector:validatedRobotPercepts", validatedRobotPercepts);
        PLOT("module:YoloRobotDetector:declinedRobotPercepts", abs(validatedRobotPercepts - robotPercepts));
      }
    }
  }
}

void YoloRobotDetector::class_init()
{
  std::string filename = std::string(File::getBHDir()) + "/Config/classificator_rgb_mini_weighted_rc22.tflite";

  // Load the model
  class_model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());
  TFLITE_MINIMAL_CHECK(class_model != nullptr);

  // Build the interpreter
  tflite::InterpreterBuilder builder(*class_model, resolver);
  builder(&class_interpreter);
  TFLITE_MINIMAL_CHECK(class_interpreter != nullptr);
  class_interpreter->SetNumThreads(1);

  // Allocate memory for the tensors
  class_interpreter->AllocateTensors();

  // Check interpreter state
  //tflite::PrintInterpreterState(class_interpreter.get());
}

bool YoloRobotDetector::class_checkEstimate(const Image& image, RobotEstimate& re)
{
  int xPos = re.imageUpperLeft.x() - robotClassifierMargin;
  int yPos = re.imageUpperLeft.y() - robotClassifierMargin;
  auto size = re.imageLowerRight - re.imageUpperLeft;
  int sizeX = size.x() + 2 * robotClassifierMargin;
  int sizeY = size.y() + 2 * robotClassifierMargin;
  if (robotClassifierProject)
  {
    image.projectIntoImage(xPos, yPos, sizeX, sizeY);
  }

  STOPWATCH("YOLO-checkRobotEstimate-copyAndResize")
  {
    image.copyAndResizeAreaRGBFloat(xPos, yPos, sizeX, sizeY, 16, 48, class_interpreter->typed_tensor<float>(class_interpreter->inputs()[0]));
  }


  float* output = class_interpreter->typed_tensor<float>(class_interpreter->outputs()[0]);
  STOPWATCH("YOLO-checkRobotEstimate-runNet")
  {
    if (class_interpreter->Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
      return true;
    }
  }
  re.validity = *output;
  return *output >= robotClassifierThreshold;
}

void YoloRobotDetector::setRobotColorFromGC()
{
  if (theOwnTeamInfo.teamNumber > 0)
  {
    ownColor = teamColorMap[theOwnTeamInfo.teamColour];
  }
  if (theOpponentTeamInfo.teamNumber > 0)
  {
    oppColor = teamColorMap[theOpponentTeamInfo.teamColour];
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

float YoloRobotDetector::colorDistance(unsigned char& r1, unsigned char& r2, unsigned char& g1, unsigned char& g2, unsigned char& b1, unsigned char& b2)
{
  // See: https://www.compuphase.com/cmetric.htm
  int rmean = ((int)r1 + (int)r2) / 2;
  int r = (int)r1 - (int)r2;
  int g = (int)g1 - (int)g2;
  int b = (int)b1 - (int)b2;
  return std::sqrt(static_cast<float>((((512 + rmean) * r * r) >> 8) + 4 * g * g + (((767 - rmean) * b * b) >> 8)));
}

float YoloRobotDetector::colorDistanceHsl(short int& h1, float& s1, float& l1, short int& h2, float& s2, float& l2)
{
  // the hardcoded SL values act as a color amplification
  unsigned char r1, g1, b1;
  ColorModelConversions::fromHSLToRGB(h1, 1.f, .5f, r1, g1, b1);
  unsigned char r2, g2, b2;
  ColorModelConversions::fromHSLToRGB(h2, 1.f, .5f, r2, g2, b2);

  return colorDistance(r1, r2, g1, g2, b1, b2);
}

Vector2f YoloRobotDetector::getInitialCheckpoint(int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY, int& gridSampleSizeVar, float& xInterval, float& yInterval)
{
  int randRadius = static_cast<int>(round(randRadiusPercentage * (lowerRightX - upperLeftX)));

  // bbox
  int xGridSize = lowerRightX - upperLeftX - 2 * randRadius;
  int yOffset = (lowerRightY - upperLeftY) / 10;
  int yGridSize = static_cast<int>((lowerRightY - upperLeftY - yOffset) / upperFractionOfEstimate - (2 * randRadius));

  // Do not use more steps than pixels if the estimate is very small
  gridSampleSizeVar = std::min(gridSampleSize, xGridSize);
  xInterval = xGridSize / static_cast<float>(gridSampleSizeVar - 1);
  yInterval = yGridSize / static_cast<float>(gridSampleSizeVar - 1);

  // use random offsets and intervals to calculat starting point
  float xRandOffset = randRadius == 0 ? 0.f : static_cast<float>(rand() % (2 * randRadius) - randRadius);
  float yRandOffset = randRadius == 0 ? 0.f : static_cast<float>(rand() % (2 * randRadius) - randRadius);
  float checkPointY = upperLeftY + randRadius + yRandOffset + yOffset;
  Vector2f checkPoint(upperLeftX + randRadius + xRandOffset - static_cast<int>(round(xInterval)), checkPointY);
  return checkPoint;
}

static void printTeamAssignmentDebug(
    const RobotEstimate& re, float ratioOwnColor, float ratioOppColor, int upperLeftX, int upperLeftY, int lowerRightX, int lowerRightY, ColorRGBA& ownColor, ColorRGBA& oppColor)
{
  std::stringstream ssOwn;
  ssOwn << std::fixed << std::setprecision(1) << ratioOwnColor * 100.f << "%";
  std::stringstream ssOpp;
  ssOpp << std::fixed << std::setprecision(1) << ratioOppColor * 100.f << "%";

  LINE("module:YoloRobotDetector:shirtScanUpper", lowerRightX + 10, lowerRightY - 15, lowerRightX + 10, lowerRightY - 15, 10, Drawings::solidPen, ownColor);
  DRAWTEXT("module:YoloRobotDetector:shirtScanUpper", lowerRightX + 20, lowerRightY - 10, 10, ColorRGBA::green, ssOwn.str());

  LINE("module:YoloRobotDetector:shirtScanUpper", lowerRightX + 10, lowerRightY + 5, lowerRightX + 10, lowerRightY + 5, 10, Drawings::solidPen, oppColor);
  DRAWTEXT("module:YoloRobotDetector:shirtScanUpper", lowerRightX + 20, lowerRightY + 10, 10, ColorRGBA::red, ssOpp.str());

  ColorRGBA color = re.robotType == RobotEstimate::teammateRobot ? ColorRGBA::green : (re.robotType == RobotEstimate::opponentRobot ? ColorRGBA::red : ColorRGBA::black);
  if (!re.fromUpperImage)
    RECTANGLE("module:YoloRobotDetector:shirtScanUpper", upperLeftX, upperLeftY, lowerRightX, lowerRightY, 10, Drawings::solidPen, color);
}

void YoloRobotDetector::transferEstimateToUpper(RobotEstimate& re, int& upperLeftX, int& upperLeftY, int& lowerRightX, int& lowerRightY)
{
  float xFraction = static_cast<float>(theImageUpper.width) / theImage.width;
  float yFraction = static_cast<float>(theImageUpper.height) / theImage.height;
  upperLeftX = static_cast<int>(round(re.imageUpperLeft.x() * xFraction));
  upperLeftY = std::max(0, static_cast<int>(round(theImageUpper.height + re.imageUpperLeft.y() * yFraction)));
  lowerRightX = static_cast<int>(round(re.imageLowerRight.x() * xFraction));
  lowerRightY = static_cast<int>(round(theImageUpper.height + re.imageLowerRight.y() * yFraction));
}

void YoloRobotDetector::updateRobotColor(RobotEstimate& re)
{
  // counter for how many pixels are estimated to belong to which team
  int numOwnColor = 0, numOppColor = 0;

  // Get the bounding box. If the estimate comes from the lower image, the robot is very near and its jersey should be seen in the upper image.
  int upperLeftX = re.imageUpperLeft.x(), upperLeftY = re.imageUpperLeft.y(), lowerRightX = re.imageLowerRight.x(), lowerRightY = re.imageLowerRight.y();
  if (!re.fromUpperImage)
  {
    transferEstimateToUpper(re, upperLeftX, upperLeftY, lowerRightX, lowerRightY);
  }

  // own and opp color as hsv
  short int ownH;
  float ownS, ownL;
  ColorModelConversions::fromRGBToHSL(ownColor.r, ownColor.g, ownColor.b, ownH, ownS, ownL);
  short int oppH;
  float oppS, oppL;
  ColorModelConversions::fromRGBToHSL(oppColor.r, oppColor.g, oppColor.b, oppH, oppS, oppL);

  int gridSampleSizeVar;
  float xInterval, yInterval;
  Vector2f checkPoint = getInitialCheckpoint(upperLeftX, upperLeftY, lowerRightX, lowerRightY, gridSampleSizeVar, xInterval, yInterval);
  float checkPointY = checkPoint.y();
  int debugPixelSize = static_cast<int>(ceil(std::max(xInterval, yInterval)));

  // two loops that iterate over a grid in the robot estimate
  for (int i = 0; i < gridSampleSizeVar; i++)
  {
    checkPoint.x() += xInterval;
    checkPoint.y() = checkPointY;
    for (int j = 0; j < gridSampleSizeVar; j++)
    {
      assignPixel(re, checkPoint, ownH, ownS, ownL, oppH, oppS, oppL, numOwnColor, numOppColor, debugPixelSize);
      checkPoint += Vector2f(0, yInterval);
    }
  }

  float ratioOwnColor = static_cast<float>(numOwnColor) / (gridSampleSizeVar * gridSampleSizeVar);
  float ratioOppColor = static_cast<float>(numOppColor) / (gridSampleSizeVar * gridSampleSizeVar);

  re.robotType = RobotEstimate::unknownRobot;

  // if enough pixels are assigned to atleast one team, consider assignment
  if (std::max(ratioOwnColor, ratioOppColor) > minAssignedPixelsPercentage)
  {
    // if atleast 2/3 belong to one team, return the according team
    if (ratioOwnColor >= 2 * ratioOppColor)
    {
      re.robotType = RobotEstimate::teammateRobot;
      re.teamAssignmentConfidence = ratioOwnColor;
    }
    else if (ratioOppColor >= 2 * ratioOwnColor)
    {
      re.robotType = RobotEstimate::opponentRobot;
      re.teamAssignmentConfidence = ratioOppColor;
    }
  }

  printTeamAssignmentDebug(re, ratioOwnColor, ratioOppColor, upperLeftX, upperLeftY, lowerRightX, lowerRightY, ownColor, oppColor);
}

bool YoloRobotDetector::isWhite(short int& pH, float& pS, float& pL)
{
  return pL > whiteLightnessMin;
}

bool YoloRobotDetector::isGray(short int& pH, float& pS, float& pL)
{
  return pS < greySaturationMax;
}

bool YoloRobotDetector::isBlack(short int& pH, float& pS, float& pL)
{
  return pL < blackLightnessMax;
}

static void printPixelAssignmentDebug(
    const RobotEstimate& re, bool showAmplifiedColor, bool showAssignmentInfo, Vector2f& checkPoint, int debugPixelSize, ColorRGBA& amplifiedPixelColor, ColorRGBA& debugColor)
{
  if (showAmplifiedColor)
    LINE("module:YoloRobotDetector:shirtScanUpper", checkPoint.x(), checkPoint.y(), checkPoint.x(), checkPoint.y(), debugPixelSize, Drawings::solidPen, amplifiedPixelColor);
  if (showAssignmentInfo)
    LINE("module:YoloRobotDetector:shirtScanUpper", checkPoint.x(), checkPoint.y(), checkPoint.x(), checkPoint.y(), 2, Drawings::solidPen, debugColor);
}

void YoloRobotDetector::assignPixel(
    const RobotEstimate& re, Vector2f& checkPoint, short& ownH, float& ownS, float& ownL, short& oppH, float& oppS, float& oppL, int& numOwnColor, int& numOppColor, int& debugPixelSize)
{
  // the debugColor shows information about how pixels are considered. If they are assigned to a team, considered field color etc.
  ColorRGBA debugColor = ColorRGBA::white;
  // the amplified color based only on the hue value or whiteness, or white if the pixel is not used for team assignment
  ColorRGBA amplifiedPixelColor = ColorRGBA::white;
  if (!theImageUpper.isOutOfImage(checkPoint.x(), checkPoint.y(), 4))
  {
    Image::Pixel p = theImageUpper[static_cast<int>(checkPoint.y())][static_cast<int>(checkPoint.x())];

    // if we dont look at the field, try to estimate the team for this point
    if (theFieldColorsUpper.isPixelFieldColor(p.y, p.cb, p.cr))
    {
      debugColor = ColorRGBA::darkgreen;
    }
    else
    {
      debugColor = ColorRGBA::black;
      short int pH;
      float pS, pL;
      ColorModelConversions::fromYCbCrToHSL(p.y, p.cb, p.cr, pH, pS, pL);

      if (isBlack(pH, pS, pL))
      {
        if (theOwnTeamInfo.teamColour == TEAM_BLACK || theOwnTeamInfo.teamColour == TEAM_GRAY)
        {
          numOwnColor += 1;
          debugColor = ColorRGBA::green;
          amplifiedPixelColor = ColorRGBA::black;
        }
        else if (theOpponentTeamInfo.teamColour == TEAM_BLACK || theOpponentTeamInfo.teamColour == TEAM_GRAY || acceptBlackOpponent)
        {
          numOppColor += 1;
          debugColor = ColorRGBA::red;
          amplifiedPixelColor = ColorRGBA::black;
        }
      }
      else if (isWhite(pH, pS, pL))
      {
        if (theOwnTeamInfo.teamColour == TEAM_WHITE || theOwnTeamInfo.teamColour == TEAM_GRAY)
        {
          numOwnColor += 1;
          debugColor = ColorRGBA::green;
          amplifiedPixelColor = ColorRGBA::gray;
        }
        else if (theOpponentTeamInfo.teamColour == TEAM_WHITE || theOpponentTeamInfo.teamColour == TEAM_GRAY)
        {
          numOppColor += 1;
          debugColor = ColorRGBA::red;
          amplifiedPixelColor = ColorRGBA::gray;
        }
      }
      else if (isGray(pH, pS, pL))
      {
        if (theOwnTeamInfo.teamColour == TEAM_GRAY)
        {
          numOwnColor += 1;
          debugColor = ColorRGBA::green;
          amplifiedPixelColor = ColorRGBA::gray;
        }
        else if (theOpponentTeamInfo.teamColour == TEAM_GRAY)
        {
          numOppColor += 1;
          debugColor = ColorRGBA::red;
          amplifiedPixelColor = ColorRGBA::gray;
        }
      }
      else
      {
        // we divide the distance by the max allowed distance. We use this ratio to get a normalized distance that should be less than 1 to be valid.
        float diffToOwnColor = colorDistanceHsl(pH, pS, pL, ownH, ownS, ownL) / maxColorDistance;
        float diffToOppColor = colorDistanceHsl(pH, pS, pL, oppH, oppS, oppL) / maxColorDistance;
        unsigned char pR, pG, pB;
        // the hardcoded SL values act as a color amplification
        ColorModelConversions::fromHSLToRGB(pH, 1.f, .5f, pR, pG, pB);
        debugColor = ColorRGBA::magenta;
        // team assignment should not be due to chance, so the pixel color should not just be in the middle of the two colors
        if (std::abs(diffToOwnColor - diffToOppColor) > minColorConfidence)
        {
          if (diffToOwnColor < diffToOppColor && diffToOwnColor < 1 && !(theOwnTeamInfo.teamColour == TEAM_BLACK || theOwnTeamInfo.teamColour == TEAM_GRAY || theOwnTeamInfo.teamColour == TEAM_WHITE))
          {
            numOwnColor += 1;
            debugColor = ColorRGBA::green;
            amplifiedPixelColor = ColorRGBA(pR, pG, pB);
          }
          else if (diffToOppColor < 1 && !(theOpponentTeamInfo.teamColour == TEAM_BLACK || theOpponentTeamInfo.teamColour == TEAM_GRAY || theOpponentTeamInfo.teamColour == TEAM_WHITE))
          {
            numOppColor += 1;
            debugColor = ColorRGBA::red;
            amplifiedPixelColor = ColorRGBA(pR, pG, pB);
          }
        }
      }
    }
  }
  printPixelAssignmentDebug(re, showAmplifiedColor, showAssignmentInfo, checkPoint, debugPixelSize, amplifiedPixelColor, debugColor);
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
