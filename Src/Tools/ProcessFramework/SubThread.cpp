/**
 * @file SubThread.cpp
 *
 * Implementation of classes SubThread and SuperThread.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#include "SubThread.h"
#include "Tools/Debugging/Modify.h"
#include <taskflow/taskflow.hpp>
#include <taskflow/algorithm/for_each.hpp>
#include <mutex>
#include <condition_variable>
#include "ExecutorObserver.h"

SubThread::SubThread(SuperThread& superThread) : superThread(&superThread), timingManager(&superThread.timingManager)
{
  sender.setSize(5200000, 100000);
  threadName = superThread.getThreadName() + "Worker";

  // copy debug requests from super to sub threads
  for (int i = 0; i < this->superThread->debugRequestTable.currentNumberOfDebugRequests; i++)
    debugRequestTable.addRequest(this->superThread->debugRequestTable.debugRequests[i]);

  // copy streamHandler data?

  Thread<ProcessBase>::setName(threadName);
  BH_TRACE_INIT(threadName.c_str());
}

SubThread::~SubThread()
{
  BH_TRACE_TERM;
}

Global SubThread::getGlobals()
{
  Global global = superThread->getGlobals();
  global.debugOut = &sender.out;
  global.timingManager = &timingManager;
  global.debugRequestTable = &debugRequestTable;
  global.streamHandler = &streamHandler;
  return global;
}

bool SubThread::handleMessage(InMessage& message)
{
  switch (message.getMessageID())
  {
  case idDebugRequest:
  {
    DebugRequest debugRequest;
    message.bin >> debugRequest;
    this->debugRequestTable.addRequest(debugRequest);
    return true;
  }
  default:
    return false;
  }
}

void SubThread::moveMessages(MessageQueue& processSender)
{
  sender.moveAllMessages(processSender);

  if (debugRequestTable.poll && debugRequestTable.notYetPolled("automated requests:StreamSpecification"))
    OUTPUT(idDebugResponse, text, "automated requests:StreamSpecification" << debugRequestTable.isActive("automated requests:StreamSpecification"));
  const bool active = debugRequestTable.isActive("automated requests:StreamSpecification");
  if (active && (debugRequestTable.disable("automated requests:StreamSpecification"), true))
  {
    this->superThread->streamHandler << streamHandler;
  }

  // super thread => sub threads
  this->superThread->debugRequestTable.propagateDisabledRequests(debugRequestTable);
  debugRequestTable.clearDisabledRequests();
}

void SubThread::beforeRun()
{
  // is set in Process::processMain() usually
  debugRequestTable.poll = this->superThread->debugRequestTable.poll;
}

void SubThread::afterRun()
{
  // sub threads => super thread
  debugRequestTable.propagateDisabledRequests(this->superThread->debugRequestTable);
}

SuperThread::SuperThread(MessageQueue& debugIn, MessageQueue& debugOut, Settings& settings, std::string configFile)
    : Process(debugIn, debugOut, settings), configFile(std::move(configFile))
{
  InMapFile stream(this->configFile);
  ASSERT(stream.exists());
  stream >> config;
}

SuperThread::~SuperThread() = default;

void SuperThread::setNumOfSubthreads(unsigned n)
{
  executor.reset();
  subthreads.resize(n);

  // Construct and destruct subthreads in their worker instances
  class SubThreadWorkerInterface : public tf::WorkerInterface
  {
  public:
    SubThreadWorkerInterface(SuperThread* superThread) : superThread(superThread) {}

    virtual void scheduler_prologue(tf::Worker& worker) override
    {
      ASSERT(!superThread->subthreads.at(worker.id()));
      superThread->subthreads.at(worker.id()) = std::make_unique<SubThread>(*superThread);

      std::unique_lock<std::mutex> l(mutex);
      if (++n == superThread->subthreads.size())
        cond.notify_one();
    }
    virtual void scheduler_epilogue(tf::Worker& worker, std::exception_ptr ptr) override
    {
      ASSERT(superThread->subthreads.at(worker.id()));
      superThread->subthreads.at(worker.id()).reset();

      {
        std::unique_lock<std::mutex> l(mutex);
        --n;
      }
      if (ptr)
        std::rethrow_exception(ptr);
    }

    void wait()
    {
      std::unique_lock<std::mutex> lock(mutex);
      cond.wait(lock,
          [&]()
          {
            return n == superThread->subthreads.size();
          });
    }

  private:
    SuperThread* superThread;

    std::mutex mutex;
    std::condition_variable cond;
    size_t n = 0;
  };

  std::shared_ptr<SubThreadWorkerInterface> workerInterface = std::make_shared<SubThreadWorkerInterface>(this);
  executor = std::make_unique<tf::Executor>(n, workerInterface);
  workerInterface->wait();
}

void SuperThread::run(tf::Taskflow& tf)
{
  if (configFile == "cognition.cfg")
    MODIFY_ONCE("threads:cognition:config", config);
  else
    MODIFY_ONCE("threads:motion:config", config);

  if (!executor || config.numWorkers != static_cast<unsigned int>(executor->num_workers()))
    setNumOfSubthreads(config.numWorkers);

  DEBUG_RESPONSE_ONCE("threads:restart") setNumOfSubthreads(threads);
  DEBUG_RESPONSE_ONCE("threads:dumpTaskflowGraph") OUTPUT_TEXT(tf.dump());

  DEBUG_RESPONSE("threads:observeTaskflow")
  {
    if (!observer)
      observer = executor->make_observer<ExecutorObserver>(getThreadName());
  }
  DEBUG_RESPONSE_ONCE("threads::observeTaskflowOnce") observer = executor->make_observer<ExecutorObserver>(getThreadName());

  observeFunction("run",
      [&]
      {
        executor->run(tf).wait();
      });

  DEBUG_RESPONSE_NOT("threads:observeTaskflow")
  {
    if (observer)
    {
      // convert observer data to json in separate thread
      observerMsgpackFuture = std::async(std::launch::async,
          [observer = this->observer]()
          {
            return nlohmann::json::to_msgpack(observer->dumpJson());
          });

      executor->remove_observer(observer);
      observer.reset();
    }

    if (observerMsgpackFuture.valid() && observerMsgpackFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      observerMsgpack = observerMsgpackFuture.get();
      observerMsgpackIt = observerMsgpack.begin();
    }

    // send data in chunks
    static constexpr ptrdiff_t maxMessageSize{100000};
    if (observerMsgpack.size() > 0)
    {
      sendMsgpackFinished = false;

      auto begin = observerMsgpackIt;
      auto end = (std::distance(begin, observerMsgpack.end()) > maxMessageSize) ? std::next(begin, maxMessageSize) : observerMsgpack.end();

      std::vector<uint8_t> sendData{std::make_move_iterator(begin), std::make_move_iterator(end)};
      STREAM_EXT(Global::getDebugOut().bin, sendData);
      Global::getDebugOut().finishMessage(idExecutorObservings);

      if (end == observerMsgpack.end())
      {
        observerMsgpack.clear();
        observerMsgpack.shrink_to_fit();
      }

      observerMsgpackIt = end;
    }
    else if (!sendMsgpackFinished)
    {
      // send empty message to acknowledge complete transmission
      STREAM_EXT(Global::getDebugOut().bin, observerMsgpack);
      Global::getDebugOut().finishMessage(idExecutorObservings);
      sendMsgpackFinished = true;
    }
  }
}

bool SuperThread::handleMessage(InMessage& message)
{
  for (auto& subthread : subthreads)
  {
    subthread->handleMessage(message);
    message.resetReadPosition();
  }
  return Process::handleMessage(message);
}

void SuperThread::observeFunction(std::string name, std::function<void()> f)
{
  if (observer)
    return observer->observeFunction(std::move(name), f);
  else
    return f();
}

void SuperThread::moveMessages(MessageQueue& processSender)
{
  for (auto& subthread : subthreads)
    subthread->moveMessages(processSender);

  debugRequestTable.clearDisabledRequests();
}

void SuperThread::beforeRun()
{
  for (auto& subthread : subthreads)
    subthread->beforeRun();
}

void SuperThread::afterRun()
{
  for (auto& subthread : subthreads)
    subthread->afterRun();

  observeFunction("copyUsedRepresentations",
      [&]
      {
        blackboard.copyUsedRepresentations();
      });
}
