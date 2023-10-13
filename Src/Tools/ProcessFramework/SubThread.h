/**
 * @file SubThread.h
 *
 * Contains the definition of classes SubThread and SuperThread.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "ExecutorObserver.h"

#include "Process.h"
#include <memory>
#include "Tools/Streams/AutoStreamable.h"
#include <future>

#define INIT_SUPERTHREAD_DEBUGGING(configFile) \
  SuperThread(theDebugReceiver, theDebugSender, configFile), theDebugReceiver(this, "Receiver.MessageQueue.O"), theDebugSender(this, "Sender.MessageQueue.S")

class SubThread;

class SuperThread : public Process
{
public:
  STREAMABLE(SuperThreadConfiguration,,
      (unsigned)(0) numWorkers
  );

  SuperThread(MessageQueue& debugIn, MessageQueue& debugOut, std::string configFile);
  SuperThread(const SuperThread&) = delete;
  SuperThread& operator=(const SuperThread&) = delete;

  void run(tf::Taskflow&);
  void moveMessages(MessageQueue&);
  void beforeRun();
  void afterRun();

protected:
  virtual bool handleMessage(InMessage&);

  template <typename F> inline auto observeFunction(std::string name, F f)
  {
    if (observer)
      return observer->observeFunction(std::move(name), f);
    else
      return f();
  }

private:
  void setNumOfSubthreads(unsigned);

  std::vector<std::unique_ptr<SubThread>> subthreads;
  std::unique_ptr<tf::Executor> executor;

  std::shared_ptr<ExecutorObserver> observer;
  std::future<std::vector<uint8_t>> observerMsgpackFuture;
  std::vector<uint8_t> observerMsgpack;
  std::vector<uint8_t>::iterator observerMsgpackIt;
  bool sendMsgpackFinished = true;
  int threads = std::thread::hardware_concurrency();

  const std::string configFile;
  SuperThreadConfiguration config;
};

class SubThread
{
public:
  SubThread(SuperThread&);
  ~SubThread();
  SubThread(const SubThread&) = delete;
  SubThread& operator=(const SubThread&) = delete;

  bool handleMessage(InMessage&);
  void moveMessages(MessageQueue&);
  void beforeRun();
  void afterRun();

private:
  TimingManager timingManager;
  DebugRequestTable debugRequestTable;
  StreamHandler streamHandler;

  std::string threadName;

  SuperThread* superThread;
  MessageQueue sender;
  MessageQueue receiver;
};
