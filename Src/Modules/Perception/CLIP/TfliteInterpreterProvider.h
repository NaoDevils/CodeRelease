/**
 * @file TfliteInterpreterProvider.h
 *
 * This modules provides the tflite interpreter.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/TfliteInterpreter.h"

MODULE(TfliteInterpreterProvider,
  PROVIDES_WITHOUT_MODIFY(BallPerceptTfliteInterpreter),
  LOADS_PARAMETERS(,
    (std::string)("") ballCNN
  )
);

class TfliteInterpreterProvider : public TfliteInterpreterProviderBase
{
private:
  void update(BallPerceptTfliteInterpreter& ballPerceptTfliteInterpreter);
};
