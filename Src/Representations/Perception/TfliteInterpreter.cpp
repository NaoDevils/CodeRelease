#include "TfliteInterpreter.h"
#include <taskflow/taskflow.hpp>
#include "Platform/File.h"

void TfliteInterpreter::updateInterpreters(const tf::Executor& executor, const std::function<std::unique_ptr<tflite::Interpreter>(const tflite::FlatBufferModel&, size_t thread)>& func)
{
  this->executor = &executor;

  interpreters.resize(executor.num_workers());

  for (size_t thread = 0; thread < interpreters.size(); ++thread)
  {
    if (!interpreters[thread])
    {
      interpreters[thread] = func(*model, thread);
    }
  }
}

void TfliteInterpreter::loadModel(const std::string& filename)
{
  if (filename != this->filename)
  {
    this->filename = filename;

    const std::string path = std::string(File::getBHDir()) + "/Config/" + filename;

    model = tflite::FlatBufferModel::BuildFromFile(path.c_str());

    for (auto& interpreter : interpreters)
      interpreter.reset();
  }
}

tflite::Interpreter& TfliteInterpreter::getInterpreter(size_t thread) const
{
  return *interpreters.at(thread).get();
}

tflite::Interpreter& TfliteInterpreter::getInterpreter() const
{
  return *interpreters.at(executor->this_worker_id()).get();
}
