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
#include "Platform/BHAssert.h"
#include <functional>

namespace tf
{
  class Executor;
}
namespace tflite
{
  class Interpreter;
  class FlatBufferModel;
} // namespace tflite

struct TfliteInterpreter : public Streamable
{
public:
  TfliteInterpreter();
  TfliteInterpreter(TfliteInterpreter&&) noexcept;
  ~TfliteInterpreter();

  void updateInterpreters(const tf::Executor& executor, const std::function<std::unique_ptr<tflite::Interpreter>(const tflite::FlatBufferModel&, size_t thread)>& func);
  void loadModel(const std::string& filename);

  tflite::Interpreter& getInterpreter() const;
  tflite::Interpreter& getInterpreter(size_t index) const;

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

private:
  std::string filename = "";
  std::vector<std::unique_ptr<tflite::Interpreter>> interpreters;
  std::unique_ptr<tflite::FlatBufferModel> model;
  const tf::Executor* executor;
};

struct BallPerceptTfliteInterpreter : public TfliteInterpreter
{
};

struct SplittedTfliteInterpreter : public Streamable
{
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
  }

  std::vector<TfliteInterpreter> layers;
};
