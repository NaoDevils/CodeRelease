/**
 * @file RefereeKeypointsDetector.h
 * 
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "RefereeKeypointsDetector.h"


#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif

#include "Tools/ImageProcessing/stb_image.h"
#include "Tools/ImageProcessing/stb_image_write.h"

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <filesystem>


MAKE_MODULE(RefereeKeypointsDetector, perception);

void RefereeKeypointsDetector::update(RefereeKeypoints& refereeKeypoints)
{
  DECLARE_DEBUG_DRAWING("module:RefereeKeypointsDetector:imageFrame", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:RefereeKeypointsDetector:keypoints", "drawingOnImage");
  DECLARE_DEBUG_RESPONSE("module:RefereeKeypointsDetector:logPNG");

  if (!initialized && theFrameInfo.time > 0)
  {
    initClassifier();
  }

  if (!initialized)
    return;

  if (theVisualRefereeBehaviorSymbols.state != VisualRefereeBehaviorSymbols::capture && theVisualRefereeBehaviorSymbols.state != VisualRefereeBehaviorSymbols::look)
    return;

  int input_tensor = interpreter->inputs()[0];
  //auto tensor = applyRobotPatchOnImage<unsigned char>(re, input_tensor);


  Vector2i minInputFrame = theVisualRefereeBehaviorSymbols.refereePositionInImage[0].cwiseMin(theVisualRefereeBehaviorSymbols.refereePositionInImage[1]);
  Vector2i maxInputFrame = theVisualRefereeBehaviorSymbols.refereePositionInImage[2].cwiseMax(theVisualRefereeBehaviorSymbols.refereePositionInImage[3]);
  Vector2i sizeInputFrame = maxInputFrame - minInputFrame;

  int inputFrameSize = sizeInputFrame.y();
  Vector2i inputFrame((minInputFrame.x() + maxInputFrame.x() - inputFrameSize) / 2, minInputFrame.y());

  auto tensor = applyPatchOnImage<float>(input_tensor, inputFrame.x(), inputFrame.y(), inputFrameSize, inputFrameSize);
  input_patch_float = std::vector<float>{tensor, tensor + input_height * input_width * input_channels};

  {
    Stopwatch s2("RefereeKeypointsDetector-runNet");
    if (interpreter->Invoke() != kTfLiteOk)
    {
      OUTPUT_ERROR("Failed to invoke tflite!");
      return;
    }
  }

  // decode movenet output

  const Vector2f patchSize(inputFrameSize, inputFrameSize);
  const Vector2i patchPos(inputFrame);

  std::vector<float> heatmaps;
  std::vector<float> centers;
  std::vector<float> regs;
  std::vector<float> offsets;

  std::map<std::string, std::vector<int>> output_shapes;
  std::vector<int> output_tensors = interpreter->outputs();
  std::sort(output_tensors.begin(), output_tensors.end());

  STOPWATCH("RefereeKeypointsDetector:postprocessing")
  {
    for (size_t i = 0; i < output_tensors.size(); i++)
    {
      std::vector<int> output_shape;
      TfLiteIntArray* output_dims = interpreter->tensor(output_tensors[i])->dims;
      int output_length = output_dims->size;
      int output_size = 1;
      for (int j = 0; j < output_length; j++)
      {
        output_shape.push_back(output_dims->data[j]);
        output_size *= output_dims->data[j];
      }
      float* result = interpreter->typed_tensor<float>(output_tensors[i]);
      if (i == 0)
      {
        output_shapes["heatmaps"] = output_shape;
        heatmaps.assign(result, result + output_size);
      }
      else if (i == 1)
      {
        output_shapes["centers"] = output_shape;
        centers.assign(result, result + output_size);
      }
      else if (i == 2)
      {
        output_shapes["regs"] = output_shape;
        regs.assign(result, result + output_size);
      }
      else
      {
        output_shapes["offsets"] = output_shape;
        offsets.assign(result, result + output_size);
      }
    }

    Vector2i center;
    int maxIndex = static_cast<int>(std::distance(centers.begin(), std::max_element(centers.begin(), centers.end())));
    center.y() = maxIndex / output_shapes["centers"][3];
    center.x() = maxIndex % output_shapes["centers"][2];

    for (int keypoint = 0; keypoint < static_cast<int>(refereeKeypoints.position.size()); keypoint++)
    {
      Vector2i reg_origin = Vector2i::Zero();
      std::vector<int> index;
      index = {0, keypoint * 2, center.y(), center.x()};
      reg_origin.x() = static_cast<int>(getElement(index, regs, output_shapes["regs"]) + 0.5f);

      index = {0, keypoint * 2 + 1, center.y(), center.x()};
      reg_origin.y() = static_cast<int>(getElement(index, regs, output_shapes["regs"]) + 0.5f);

      Vector2i reg = Vector2i::Zero();
      reg.x() = reg_origin.x() + center.x();
      reg.y() = reg_origin.y() + center.y();

      Eigen::MatrixXi regMatrixX = Eigen::MatrixXi::Constant(output_shapes["regs"][2], output_shapes["regs"][3], reg.x());
      Eigen::MatrixXi regMatrixY = Eigen::MatrixXi::Constant(output_shapes["regs"][2], output_shapes["regs"][3], reg.y());

      if (rangeWeightX.rows() != output_shapes["regs"][2] || rangeWeightX.cols() != output_shapes["regs"][3])
      {
        rangeWeightX.resize(output_shapes["regs"][2], output_shapes["regs"][3]);
        rangeWeightY.resize(output_shapes["regs"][2], output_shapes["regs"][3]);

        for (int row = 0; row < rangeWeightX.rows(); ++row)
        {
          for (int col = 0; col < rangeWeightX.cols(); ++col)
          {
            rangeWeightX(row, col) = row; //TODO check
          }
        }
        rangeWeightY = rangeWeightX.transpose();
      }

      Eigen::MatrixXi tempRegMatrixX;
      Eigen::MatrixXi tempRegMatrixY;
      Eigen::MatrixXf tempRegMatrix;

      tempRegMatrixX = (rangeWeightX - regMatrixX);
      tempRegMatrixX = tempRegMatrixX.cwiseAbs2();

      tempRegMatrixY = (rangeWeightY - regMatrixY);
      tempRegMatrixY = tempRegMatrixY.cwiseAbs2();

      tempRegMatrix = (tempRegMatrixX + tempRegMatrixY).cast<float>();
      //tempRegMatrix = tempRegMatrix.cwiseSqrt();
      tempRegMatrix = tempRegMatrix.array().sqrt();
      tempRegMatrix.array() += 1.8f;

      for (int row = 0; row < tempRegMatrix.rows(); ++row)
      {
        for (int col = 0; col < tempRegMatrix.cols(); ++col)
        {
          index = {0, keypoint, col, row};
          tempRegMatrix(row, col) = getElement(index, heatmaps, output_shapes["heatmaps"]) / tempRegMatrix(row, col); //TODO check
        }
      }

      Eigen::Index maxRow, maxCol;
      tempRegMatrix.maxCoeff(&maxRow, &maxCol);
      reg.y() = static_cast<int>(maxCol);
      reg.x() = static_cast<int>(maxRow);
      //reg.y() = maxIndex / output_shapes["heatmaps"][3];
      //reg.x() = maxIndex % output_shapes["heatmaps"][2];


      index = {0, keypoint, reg.y(), reg.x()};
      float score = getElement(index, heatmaps, output_shapes["heatmaps"]);

      Vector2f offset;
      index = {0, keypoint * 2, reg.y(), reg.x()};
      offset.x() = getElement(index, offsets, output_shapes["offsets"]);
      index = {0, keypoint * 2 + 1, reg.y(), reg.x()};
      offset.y() = getElement(index, offsets, output_shapes["offsets"]);

      Vector2f res;
      res.x() = (reg.x() + offset.x()) / output_shapes["heatmaps"][2];
      res.y() = (reg.y() + offset.y()) / output_shapes["heatmaps"][3];


      // res.x() = static_cast<float>(reg.x()) / output_shapes["heatmaps"][2] + offset.x();
      // res.y() = static_cast<float>(reg.y()) / output_shapes["heatmaps"][3] + offset.y();

      // float fa = reg.x() + offset.x();
      // float fb = reg.y() + offset.y();
      // float fx = reg.x() / output_shapes["heatmaps"][2] + offset.x();
      // float fy = reg.y() / output_shapes["heatmaps"][3] + offset.y();

      res.x() *= patchSize.x();
      res.y() *= patchSize.y();
      // fx *= patchSize.x();
      // fy *= patchSize.y();

      if (smoothKeypointsFactor > 0.f && smoothKeypointsFactor < 1.f)
      {
        refereeKeypoints.position[keypoint] = smoothKeypointsFactor * refereeKeypoints.position[keypoint] + (1.f - smoothKeypointsFactor) * res;
        refereeKeypoints.confidence[keypoint] = smoothKeypointsFactor * refereeKeypoints.confidence[keypoint] + (1.f - smoothKeypointsFactor) * score;
      }
      else
      {
        refereeKeypoints.position[keypoint] = res;
        refereeKeypoints.confidence[keypoint] = score;
      }
      // float f = fx + fy+ fa+ fb;
    }

    if (useCOCO)
    {
      const std::array<size_t, 13> coco_order = {3, 4, 2, 5, 1, 6, 0, 8, 7, 10, 9, 12, 11};
      reorder(refereeKeypoints.position, coco_order);
    }

    COMPLEX_DRAWING("module:RefereeKeypointsDetector:keypoints")
    {
      Vector2f scaledCenter;
      scaledCenter.x() = patchPos.x() + (center.x() / static_cast<float>(output_shapes["centers"][2])) * patchSize.x();
      scaledCenter.y() = patchPos.y() + (center.y() / static_cast<float>(output_shapes["centers"][3])) * patchSize.y();

      CIRCLE("module:RefereeKeypointsDetector:keypoints", scaledCenter.x(), scaledCenter.y(), 5, 1, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA::blue);
      // Debug color 17
      /*ColorRGBA colors[] = {ColorRGBA::red, ColorRGBA::black,ColorRGBA::black,ColorRGBA::black,ColorRGBA::black,
                            ColorRGBA::blue,ColorRGBA::yellow,ColorRGBA::magenta,ColorRGBA::orange,
                            ColorRGBA::red,ColorRGBA::green,ColorRGBA::darkgreen,ColorRGBA::darkgreen,
                            ColorRGBA::cyan,ColorRGBA::white,ColorRGBA::brown,ColorRGBA::purple};
      */

      // clang-format off
      
      // Debug color 13
      ColorRGBA colors[] = {
          ColorRGBA::red,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::green,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::red,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::yellow,
          ColorRGBA::yellow};
      // clang-format on

      for (size_t i = 0; i < refereeKeypoints.position.size(); i++)
      {
        ColorRGBA color = colors[i];

        if (refereeKeypoints.confidence[i] < minConfidence)
          color = ColorRGBA::black;
        //if (i % 2 == 1) color = ColorRGBA::blue;
        CIRCLE("module:RefereeKeypointsDetector:keypoints", refereeKeypoints.position[i].x() + patchPos.x(), refereeKeypoints.position[i].y() + patchPos.y(), 5, 1, Drawings::solidPen, color, Drawings::noBrush, color); // TODO: 80
      }
      int size = 2;

      /* 13 Keypoints:
      0: right hand
      1: right elbow
      2: right shoulder
      3: nose
      4: left shoulder
      5: left elbow
      6: left hand
      7: right hip
      8: left hip
      9: right knee
      10:left knee
      11:right ankle
      12:left ankle
      */
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[0].x() + patchPos.x(),
          refereeKeypoints.position[0].y() + patchPos.y(),
          refereeKeypoints.position[1].x() + patchPos.x(),
          refereeKeypoints.position[1].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[1].x() + patchPos.x(),
          refereeKeypoints.position[1].y() + patchPos.y(),
          refereeKeypoints.position[2].x() + patchPos.x(),
          refereeKeypoints.position[2].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[4].x() + patchPos.x(),
          refereeKeypoints.position[4].y() + patchPos.y(),
          refereeKeypoints.position[5].x() + patchPos.x(),
          refereeKeypoints.position[5].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[5].x() + patchPos.x(),
          refereeKeypoints.position[5].y() + patchPos.y(),
          refereeKeypoints.position[6].x() + patchPos.x(),
          refereeKeypoints.position[6].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[2].x() + patchPos.x(),
          refereeKeypoints.position[2].y() + patchPos.y(),
          refereeKeypoints.position[4].x() + patchPos.x(),
          refereeKeypoints.position[4].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[2].x() + patchPos.x(),
          refereeKeypoints.position[2].y() + patchPos.y(),
          refereeKeypoints.position[7].x() + patchPos.x(),
          refereeKeypoints.position[7].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[4].x() + patchPos.x(),
          refereeKeypoints.position[4].y() + patchPos.y(),
          refereeKeypoints.position[8].x() + patchPos.x(),
          refereeKeypoints.position[8].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[7].x() + patchPos.x(),
          refereeKeypoints.position[7].y() + patchPos.y(),
          refereeKeypoints.position[8].x() + patchPos.x(),
          refereeKeypoints.position[8].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[8].x() + patchPos.x(),
          refereeKeypoints.position[8].y() + patchPos.y(),
          refereeKeypoints.position[10].x() + patchPos.x(),
          refereeKeypoints.position[10].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[7].x() + patchPos.x(),
          refereeKeypoints.position[7].y() + patchPos.y(),
          refereeKeypoints.position[9].x() + patchPos.x(),
          refereeKeypoints.position[9].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[10].x() + patchPos.x(),
          refereeKeypoints.position[10].y() + patchPos.y(),
          refereeKeypoints.position[12].x() + patchPos.x(),
          refereeKeypoints.position[12].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);

      LINE("module:RefereeKeypointsDetector:keypoints",
          refereeKeypoints.position[9].x() + patchPos.x(),
          refereeKeypoints.position[9].y() + patchPos.y(),
          refereeKeypoints.position[11].x() + patchPos.x(),
          refereeKeypoints.position[11].y() + patchPos.y(),
          size,
          Drawings::solidPen,
          ColorRGBA::black);
    }
  }
}

void RefereeKeypointsDetector::reorder(std::array<Vector2f, 13>& v, std::array<size_t, 13> const& order)
{
  for (size_t s = 1, d; s < order.size(); ++s)
  {
    for (d = order[s]; d < s; d = order[d])
      ;
    if (d == s)
      while (d = order[d], d != s)
        std::swap(v[s], v[d]);
  }
}

float RefereeKeypointsDetector::getElement(std::vector<int>& index, const std::vector<float>& vec, const std::vector<int>& shape)
{
  ASSERT(index.size() == shape.size());

  int elementIndex = 0;
  for (size_t i = 0; i < index.size(); i++)
  {
    int sum = 1;
    for (size_t j = i + 1; j < index.size(); j++)
    {
      sum *= shape[j];
    }
    elementIndex += index[i] * sum;
  }

  return vec.at(elementIndex);
}


void RefereeKeypointsDetector::initClassifier()
{
  std::string filename = std::string(File::getBHDir()) + "/Config/" + std::string(modelName);

  // Load the model
  model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());
  VERIFY(model);
  std::map<std::string, std::string> metadata = model->ReadAllMetadata();
  //VERIFY(metadata["min_runtime_version"].rfind("1.13.", 0) == 0); // Version check 1.13.X


  // Build the interpreter
  tflite::InterpreterBuilder builder(*model, resolver);
  builder(&interpreter);
  VERIFY(interpreter);
  interpreter->SetNumThreads(numThreads);

  // Resize input tensors
  int input_tensor = interpreter->inputs()[0];
  input_type = interpreter->tensor(input_tensor)->type;
  TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
  const int input_batch = 1;
  input_height = input_dims->data[3]; // 1
  input_width = input_dims->data[2]; // 2
  input_channels = input_dims->data[1]; // 3

  interpreter->ResizeInputTensor(input_tensor, {input_batch, input_channels, input_height, input_width});
  input_patch_float.reserve(input_batch * input_height * input_width * input_channels);

  // Allocate memory for the tensors
  VERIFY(interpreter->AllocateTensors() == kTfLiteOk);

  initialized = true;
}

template <typename T> T* RefereeKeypointsDetector::applyPatchOnImage(int input_tensor, int xmin, int ymin, int width, int height)
{
  const Image& image = theImageUpper;
  //reSize = Vector2i(reSize.y(), reSize.y());

  T* input = interpreter->typed_tensor<T>(input_tensor);

  Vector2i min = Vector2i(xmin, ymin);
  const Vector2i size = Vector2i(width, height);
  const Vector2i max = min + size;

  image.copyAndResizeArea<true, true, true, true, true>(min, size, {input_width, input_height}, input);

  DEBUG_RESPONSE_ONCE("module:RefereeKeypointsDetector:logPNG")
  {
    std::string filepath = File::getBHDir() + std::string("/Config/Logs/PNGs/RefereeKeypointsDetector/");
    std::string filename = filepath + std::to_string(theFrameInfo.time) + ".png";

    if (filename.length() >= 255)
      OUTPUT_ERROR("Error, filename is too long: " << filename);

    std::vector<unsigned char> charPatch;
    charPatch.reserve(1 * input_height * input_width * input_channels);
    image.copyAndResizeArea<true, true, true, false, false>(min, size, {input_width, input_height}, charPatch.data());

    if (!stbi_write_png(filename.c_str(), input_height, input_width, input_channels, charPatch.data(), input_width * input_channels))
    {
      std::filesystem::create_directories(filepath);
      if (!stbi_write_png(filename.c_str(), input_height, input_width, input_channels, charPatch.data(), input_width * input_channels))
      {
        OUTPUT_ERROR("Error writing PNG: " << filename.c_str());
      }
    }
  }

  RECTANGLE("module:RefereeKeypointsDetector:imageFrame", min.x(), min.y(), max.x(), max.y(), 2, Drawings::dottedPen, ColorRGBA(255, 255, 0, 200));

  // TODO Add to copyAndResizeArea
  /*
  for (int i = 0; i < input_width * input_height * input_channels; i++)
  {
    input[i] *= 255.0f;
  }
  */
  return input;
}
