/**
 * @file TfliteInterpreter.h
 *
 * This representation contains the tflite interpreter.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 */

#pragma once

#include <memory>
#include "Tools/Streams/Streamable.h"
#include "Modules/Perception/TFlite.h"

struct TfliteInterpreter : public Streamable
{
  std::string filename = "";
  std::unique_ptr<tflite::Interpreter> interpreter = nullptr;
  std::unique_ptr<tflite::FlatBufferModel> model = nullptr;

  TfliteInterpreter() = default;
  TfliteInterpreter(const TfliteInterpreter&) = delete;
  TfliteInterpreter& operator=(const TfliteInterpreter&) = delete;

  virtual Streamable& operator=(const Streamable&) noexcept
  {
    // this representation is not copyable
    ASSERT(false);
    return *this;
  }

  virtual void serialize(In* in, Out* out)
  {
    // this representation is not streamable
    ASSERT(false);
  };
};

struct BallPerceptTfliteInterpreter : public TfliteInterpreter
{
};
