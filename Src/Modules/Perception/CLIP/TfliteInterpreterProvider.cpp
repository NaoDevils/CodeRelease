#include "TfliteInterpreterProvider.h"
#include "Platform/File.h"

void TfliteInterpreterProvider::update(BallPerceptTfliteInterpreter& ballPerceptTfliteInterpreter)
{
  if (ballPerceptTfliteInterpreter.filename != ballCNN)
  {
    auto& interpreter = ballPerceptTfliteInterpreter.interpreter;
    auto& model = ballPerceptTfliteInterpreter.model;
    ballPerceptTfliteInterpreter.filename = ballCNN;
    const std::string filename = std::string(File::getBHDir()) + "/Config/" + ballPerceptTfliteInterpreter.filename;

    // Load the model
    model = tflite::FlatBufferModel::BuildFromFile(filename.c_str());
    if (model == nullptr)
    {
      OUTPUT_ERROR("Could not create " << filename.c_str() << " model!");
      return;
    }

    // Build the interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    builder(&interpreter);
    if (interpreter == nullptr)
    {
      OUTPUT_ERROR("Could not create interpreter from " << filename.c_str() << "!");
      return;
    }

    interpreter->SetNumThreads(1);

    // Resize input tensors
    int input_tensor = interpreter->inputs()[0];
    //int output_tensor = interpreter->outputs()[0];
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
  }
}

MAKE_MODULE(TfliteInterpreterProvider, perception)
