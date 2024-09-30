#include "TfliteInterpreterProvider.h"
#include "Representations/Perception/BallPercept.h"
#include "Platform/File.h"
#include "Modules/Perception/TFlite.h"
#include "Tools/Math/Random.h"
#include <taskflow/taskflow.hpp>

TfliteInterpreterProvider::TfliteInterpreterProvider()
{
  inferenceModeInput.resize(CNN_POSITION_SIZE * CNN_POSITION_SIZE * 3);
  setInferenceModeInput(inferenceMode);
}

void TfliteInterpreterProvider::setInferenceModeInput(InferenceMode inferenceMode)
{
  if (inferenceMode == InferenceMode::randomNumbers)
  {
    for (size_t i = 0; i < inferenceModeInput.size(); i++)
    {
      inferenceModeInput[i] = static_cast<unsigned char>(random(256));
    }
  }
  else
  {
    std::fill(inferenceModeInput.begin(), inferenceModeInput.end(), (unsigned char)0);
  }
}

void TfliteInterpreterProvider::execute(tf::Subflow& subflow)
{
  this->executor = &subflow.executor();
}

void TfliteInterpreterProvider::update(BallPerceptTfliteInterpreter& ballPerceptTfliteInterpreter)
{
  ballPerceptTfliteInterpreter.loadModel(ballCNN);

  const auto buildInterpreter = [&](const tflite::FlatBufferModel& model, size_t index)
  {
    // Build the interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(model, resolver);
    std::unique_ptr<tflite::Interpreter> interpreter;
    builder(&interpreter);
    ASSERT(interpreter);

    interpreter->SetNumThreads(1);

    // Resize input tensors
    int input_tensor = interpreter->inputs()[0];
    TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
    const int input_batch = 1;
    const int input_height = input_dims->data[1];
    const int input_width = input_dims->data[2];
    const int input_channels = input_dims->data[3];
    interpreter->ResizeInputTensor(input_tensor, {input_batch, input_height, input_width, input_channels});

    // Allocate memory for the tensors
    interpreter->AllocateTensors();

    return interpreter;
  };

  ballPerceptTfliteInterpreter.updateInterpreters(*executor, buildInterpreter);

  if (inferenceMode != InferenceMode::off)
  {
    if ((inferenceMode == InferenceMode::zeros && inferenceModeInput[0] != 0) || inferenceMode == InferenceMode::randomNumbers)
    {
      setInferenceModeInput(inferenceMode);
    }

    STOPWATCH_WITH_PLOT("inferenceMode:ballPerceptTfliteInterpreter")
    {
      auto& interpreter = ballPerceptTfliteInterpreter.getInterpreter();

      int input_tensor = interpreter.inputs()[0];
      TfLiteIntArray* input_dims = interpreter.tensor(input_tensor)->dims;
      unsigned char* input = interpreter.typed_tensor<unsigned char>(input_tensor);
      std::memcpy(input, &inferenceModeInput[0], (input_dims->data[0] * input_dims->data[1] * input_dims->data[2] * input_dims->data[3]) * sizeof(unsigned char));

      if (interpreter.Invoke() != kTfLiteOk)
      {
        OUTPUT_ERROR("Failed to invoke tflite!");
      }

      float* result = interpreter.typed_output_tensor<float>(0);
      int output_tensor = interpreter.outputs()[0];
      TfLiteIntArray* output_dims = interpreter.tensor(output_tensor)->dims;
      unsigned output_size = output_dims->data[0];
      for (int j = 1; j < output_dims->size; j++)
        output_size *= output_dims->data[j];
      result_vector.assign(result, result + output_size);
    }
  }
}

void TfliteInterpreterProvider::update(SplittedTfliteInterpreter& splittedTfliteInterpreter)
{
  if (splittedTfliteInterpreter.layers.size() != splittedBallCNN.size())
    splittedTfliteInterpreter.layers.resize(splittedBallCNN.size());

  for (size_t i = 0; i < splittedBallCNN.size(); i++)
  {
    const std::string layerName = splittedBallCNN[i];
    ASSERT(!layerName.empty());

    const auto buildInterpreter = [&](const tflite::FlatBufferModel& model, size_t thread)
    {
      // Build the interpreter
      tflite::ops::builtin::BuiltinOpResolver resolver;
      tflite::InterpreterBuilder builder(model, resolver);
      std::unique_ptr<tflite::Interpreter> interpreter;
      builder(&interpreter);
      ASSERT(interpreter);

      interpreter->SetNumThreads(1);

      // Resize input tensors
      int input_tensor = interpreter->inputs()[0];
      TfLiteIntArray* input_dims = interpreter->tensor(input_tensor)->dims;
      const int input_batch = 1;
      const int input_height = input_dims->data[1];
      const int input_width = input_dims->data[2];
      const int input_channels = input_dims->data[3];
      interpreter->ResizeInputTensor(input_tensor, {input_batch, input_height, input_width, input_channels});

      if (i > 0)
      {
        const TfLiteTensor* prevOutputTensor = splittedTfliteInterpreter.layers[i - 1].getInterpreter(thread).output_tensor(0);
        if (interpreter->SetCustomAllocationForTensor(input_tensor, {prevOutputTensor->data.raw, prevOutputTensor->bytes}) != kTfLiteOk)
        {
          OUTPUT_ERROR("Failed to set custom allocation for tensor!");
        }
      }

      // Allocate memory for the tensors
      interpreter->AllocateTensors();

      return interpreter;
    };

    auto& layer = splittedTfliteInterpreter.layers.at(i);
    layer.loadModel(layerName);
    layer.updateInterpreters(*executor, buildInterpreter);
  }

  if (inferenceMode != InferenceMode::off)
  {
    result_vector_split.clear();
    if ((inferenceMode == InferenceMode::zeros && inferenceModeInput[0] != 0) || inferenceMode == InferenceMode::randomNumbers)
    {
      setInferenceModeInput(inferenceMode);
    }

    {
      Stopwatch s(WITH_PLOT("inferenceMode:splittedTfliteInterpreter"));
      {
        Stopwatch s2(WITH_PLOT("inferenceMode:splittedTfliteInterpreter:earlyExit"));
        auto& interpreter = splittedTfliteInterpreter.layers.at(0).getInterpreter();

        int input_tensor = interpreter.inputs()[0];
        TfLiteIntArray* input_dims = interpreter.tensor(input_tensor)->dims;
        unsigned char* input = interpreter.typed_tensor<unsigned char>(input_tensor);
        std::memcpy(input, &inferenceModeInput[0], (input_dims->data[0] * input_dims->data[1] * input_dims->data[2] * input_dims->data[3]) * sizeof(unsigned char));

        if (interpreter.Invoke() != kTfLiteOk)
        {
          OUTPUT_ERROR("Failed to invoke tflite!");
        }

        float* result = interpreter.typed_output_tensor<float>(0);
        result_vector_split.emplace_back(result, result + 1);
      }

      {
        Stopwatch s2(WITH_PLOT("inferenceMode:splittedTfliteInterpreter:remaining"));
        auto& interpreter = splittedTfliteInterpreter.layers.at(1).getInterpreter();
        if (interpreter.Invoke() != kTfLiteOk)
        {
          OUTPUT_ERROR("Failed to invoke tflite!");
        }

        float* result = interpreter.typed_output_tensor<float>(0);
        int output_tensor = interpreter.outputs()[0];
        TfLiteIntArray* output_dims = interpreter.tensor(output_tensor)->dims;
        unsigned output_size = output_dims->data[0];
        for (int j = 1; j < output_dims->size; j++)
          output_size *= output_dims->data[j];
        result_vector_split.emplace_back(result, result + output_size);
      }
    }
  }
}


MAKE_MODULE(TfliteInterpreterProvider, perception)
