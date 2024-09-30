/**
 * @file TfliteInterpreterProvider.h
 *
 * This modules provides the tflite interpreter.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Enum.h"
#include "Representations/Perception/TfliteInterpreter.h"

MODULE(TfliteInterpreterProvider,
  HAS_PREEXECUTION,
  PROVIDES_WITHOUT_MODIFY(BallPerceptTfliteInterpreter),
  PROVIDES_WITHOUT_MODIFY(SplittedTfliteInterpreter),
  LOADS_PARAMETERS(
    ENUM(InferenceMode,
      off,
      zeros,
      randomNumbers
    );,
    (InferenceMode)(InferenceMode::randomNumbers) inferenceMode,
    (std::string)("") ballCNN,
    (std::vector<std::string>) splittedBallCNN
  )
);

class TfliteInterpreterProvider : public TfliteInterpreterProviderBase
{
public:
  TfliteInterpreterProvider();

private:
  void execute(tf::Subflow& subflow);
  void update(BallPerceptTfliteInterpreter& ballPerceptTfliteInterpreter);
  void update(SplittedTfliteInterpreter& splittedTfliteInterpreter);

  void setInferenceModeInput(InferenceMode inferenceMode);

  const tf::Executor* executor;

  std::vector<unsigned char> inferenceModeInput;

  std::vector<float> result_vector;
  std::vector<std::vector<float>> result_vector_split;
};
