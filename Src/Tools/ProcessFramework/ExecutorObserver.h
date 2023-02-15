/**
 * @file ExecutorObserver.h
 *
 * Contains the definition of class ExecutorObserver.
 */

#pragma once

#include <nlohmann/json.hpp>
#include <regex>
#include <mutex>
#include <memory>
#include <limits>

#include <taskflow/taskflow.hpp>

/**
@class: ExecutorObserver

@brief Default executor observer to dump the execution timelines

*/
class ExecutorObserver : public tf::ObserverInterface
{
  friend class tf::Executor;

  // data structure to record each task execution
  struct Execution
  {
    const std::string name;

    std::chrono::time_point<std::chrono::steady_clock> beg;
    std::chrono::time_point<std::chrono::steady_clock> end;

    Execution(std::string name, std::chrono::time_point<std::chrono::steady_clock> b)
        : name{std::move(name)}, beg{b}, end{std::chrono::time_point<std::chrono::steady_clock>::min()}
    {
    }

    Execution(std::string name, std::chrono::time_point<std::chrono::steady_clock> b, std::chrono::time_point<std::chrono::steady_clock> e) : name{std::move(name)}, beg{b}, end{e}
    {
    }
  };

  struct TaskExecution : Execution
  {
    const size_t num_dependents;
    const size_t num_successors;

    TaskExecution(const tf::TaskView& tv, std::chrono::time_point<std::chrono::steady_clock> b)
        : Execution{tv.name(), b}, num_dependents{tv.num_dependents()}, num_successors{tv.num_successors()}
    {
    }

    TaskExecution(const tf::TaskView& tv, std::chrono::time_point<std::chrono::steady_clock> b, std::chrono::time_point<std::chrono::steady_clock> e)
        : Execution{tv.name(), b, e}, num_dependents{tv.num_dependents()}, num_successors{tv.num_successors()}
    {
    }
  };


public:
  ExecutorObserver(std::string name) : _name(std::move(name))
  {
    _thisInstanceId = _instanceId.fetch_add(1, std::memory_order_relaxed) + 1;
    {
      std::lock_guard lock{_origin_mutex};
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _origin).count() >= 1)
        _origin = std::chrono::steady_clock::now();
    }
  }

  inline nlohmann::json dumpJson() const;

  /**
  @brief dump the timelines in JSON to a std::string
  @return a JSON string
  */
  inline std::string dump();

  /**
  @brief clear the timeline data
  */
  inline void clear();

  /**
  @brief get the number of total tasks in the observer
  @return number of total tasks
  */
  inline size_t num_tasks() const;

  template <typename F> inline auto observeFunction(std::string name, F f);

private:
  inline void set_up(size_t num_workers) override final;
  inline void on_entry(tf::WorkerView worker, tf::TaskView task_view) override final;
  inline void on_exit(tf::WorkerView worker, tf::TaskView task_view) override final;

  static std::chrono::time_point<std::chrono::steady_clock> _origin;
  static std::mutex _origin_mutex;
  std::vector<std::vector<TaskExecution>> _taskExecutions;
  std::vector<Execution> _genericExecutions;
  const std::string _name;

  static std::atomic_char _instanceId;
  char _thisInstanceId = 0;
};

// Procedure: set_up
inline void ExecutorObserver::set_up(size_t num_workers)
{
  _taskExecutions.resize(num_workers);

  for (unsigned w = 0; w < _taskExecutions.size(); ++w)
  {
    _taskExecutions[w].reserve(100000);
  }

  _genericExecutions.reserve(100000);
}

// Procedure: on_entry
inline void ExecutorObserver::on_entry(tf::WorkerView wv, tf::TaskView tv)
{
  if (_taskExecutions[wv.id()].size() > 0 && _taskExecutions[wv.id()].back().end == std::chrono::time_point<std::chrono::steady_clock>::min())
  {
    _taskExecutions[wv.id()].back().end = std::chrono::steady_clock::now();
  }

  _taskExecutions[wv.id()].push_back({tv, std::chrono::steady_clock::now()});
}

// Procedure: on_exit
inline void ExecutorObserver::on_exit(tf::WorkerView wv, tf::TaskView tv)
{
  assert(_taskExecutions[wv.id()].size() > 0);

  if (_taskExecutions[wv.id()].back().end == std::chrono::time_point<std::chrono::steady_clock>::min())
  {
    _taskExecutions[wv.id()].back().end = std::chrono::steady_clock::now();
  }
}

// Function: clear
inline void ExecutorObserver::clear()
{
  for (size_t w = 0; w < _taskExecutions.size(); ++w)
  {
    _taskExecutions[w].clear();
  }
}

// Procedure: dump
inline nlohmann::json ExecutorObserver::dumpJson() const
{

  using json = nlohmann::json;

  json j;

  j += {{"name", "process_name"}, {"ph", "M"}, {"pid", _thisInstanceId}, {"args", {{"name", _name}}}};

  const std::regex taskNameRegex("([^ ]+) \\[(.+)\\]");
  for (size_t worker = 0; worker < _taskExecutions.size(); worker++)
  {
    for (const auto& execution : _taskExecutions[worker])
    {
      json entry = {
          {"ph", "X"},
          {"pid", _thisInstanceId},
          {"tid", worker + 1},
          {"ts", std::chrono::duration<double, std::micro>(execution.beg - _origin).count()},
          {"dur", std::chrono::duration<double, std::micro>(execution.end - execution.beg).count()},
      };

      std::string name = "", cat = "";
      if (std::smatch matches; std::regex_match(execution.name, matches, taskNameRegex) && matches.size() == 3)
      {
        name = matches[1];
        cat = matches[2];
      }

      entry["cat"] = cat;
      entry["name"] = name;
      entry["args"] = {{"dependents", execution.num_dependents}, {"successors", execution.num_successors}};

      j += entry;
    }
  }

  for (const auto& execution : _genericExecutions)
  {
    json entry = {
        {"ph", "X"},
        {"pid", _thisInstanceId},
        {"tid", 0},
        {"ts", std::chrono::duration<double, std::micro>(execution.beg - _origin).count()},
        {"dur", std::chrono::duration<double, std::micro>(execution.end - execution.beg).count()},
    };

    entry["cat"] = "GenericExecution";
    entry["name"] = execution.name;

    j += entry;
  }

  return j;
}

// Function: dump
inline std::string ExecutorObserver::dump()
{
  std::ostringstream oss;
  oss << dumpJson();
  return oss.str();
}

// Function: num_tasks
inline size_t ExecutorObserver::num_tasks() const
{
  const auto sumUpSize = [](size_t sum, const auto& exe)
  {
    return sum + exe.size();
  };

  return std::accumulate(_taskExecutions.begin(), _taskExecutions.end(), size_t{0}, sumUpSize) + _genericExecutions.size();
}

template <typename F> inline auto ExecutorObserver::observeFunction(std::string name, F f)
{
  auto beg = std::chrono::steady_clock::now();

  if constexpr (std::is_same<decltype(f()), void>::value)
  {
    f();

    auto end = std::chrono::steady_clock::now();

    _genericExecutions.push_back({std::move(name), beg, end});
  }
  else
  {
    auto ret = f();

    auto end = std::chrono::steady_clock::now();

    _genericExecutions.push_back({std::move(name), beg, end});

    return ret;
  }
}
