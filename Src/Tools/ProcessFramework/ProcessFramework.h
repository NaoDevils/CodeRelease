/**
 * @file Tools/ProcessFramework/ProcessFramework.h
 *
 * This file declares classes corresponding to the process framework.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include <list>
#include <memory>
#include "PlatformProcess.h"
#include "Receiver.h"
#include "Sender.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Global.h"
#ifdef TARGET_SIM
#include "Controller/RoboCupCtrl.h"
#endif

class Logger;
struct Settings;

/**
 * The class is a helper that allows to instantiate a class as an Windows process.
 * ProcessBase contains the parts that need not to be implemented as a template.
 * It will only be used by the macro MAKE_PROCESS and should never be used directly.
 */
class ProcessBase : public Thread<ProcessBase>
{
protected:
  /**
   * The main function of this Windows thread.
   */
  virtual void main() = 0;

public:
  virtual ~ProcessBase() = default;

  /**
   * The function starts the process by starting the Windows thread.
   */
  void start() { Thread<ProcessBase>::start(this, &ProcessBase::main); }

  /**
   * The functions searches for a sender with a given name.
   * @param name The name of the sender.
   * @return If the sender was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual SenderList* lookupSender(const std::string& name) = 0;

  /**
   * The functions searches for a receiver with a given name.
   * @param name The name of the receiver.
   * @return If the receiver was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual ReceiverList* lookupReceiver(const std::string& name) = 0;

  /**
   * The function returns the name of the process.
   * @return The name of the process that normally is its class name.
   */
  virtual const std::string& getName() const = 0;

  /**
   * The function returns a pointer to the process if it has the given name.
   * @param processName The name of the process that is searched for.
   * @return If the process has the required name, a pointer to it is
   *         returned. Otherwise, the function returns 0.
   */
  virtual PlatformProcess* getProcess(const std::string& processName) = 0;
};

/**
 * The class is a helper that allows to instantiate a class as an Windows process.
 * ProcessCreator contains the parts that need to be implemented as a template.
 * It will only be used by the macro MAKE_PROCESS and should never be used directly.
 */
template <class T> class ProcessFrame : public ProcessBase
{
private:
  std::string name; /**< The name of the process. */
  T process; /**< The process. */

protected:
  /**
   * The main function of this Windows thread.
   */
  virtual void main()
  {
#ifdef TARGET_SIM
    const std::string threadName = RoboCupCtrl::controller->getRobotName() + "." + name;
#else
    const std::string threadName = name;
#endif
    Thread<ProcessBase>::setName(threadName);
    process.setThreadName(threadName);

    // Call process.nextFrame if no blocking receivers are waiting
    setPriority(process.getPriority());
    process.processBase = this;
    Thread<ProcessBase>::yield(); // always leave processing time to other threads
    GlobalGuard g(process.getGlobals());
    while (isRunning())
    {
      if (process.getFirstReceiver())
        process.getFirstReceiver()->checkAllForPackages();
      bool wait = process.processMain();
      if (process.getFirstSender())
        process.getFirstSender()->sendAllUnsentPackages();
      if (wait)
        process.wait();
    }
    process.terminate();
  }

public:
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wuninitialized"
#endif

  /**
   * Note that process.setGlobals() is called before process is constructed.
   * @param name The name of the process.
   */
  ProcessFrame(const std::string& name, Logger* logger, Settings& settings) : name(name), process(settings)
  {
    process.setLogger(logger);
  }

#ifdef __clang__
#pragma clang diagnostic pop
#endif

  /**
   * The functions searches for a sender with a given name.
   * @param senderName The name of the sender.
   * @return If the sender was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual SenderList* lookupSender(const std::string& senderName)
  {
    return process.getFirstSender() ? process.getFirstSender()->lookup(name, senderName) : nullptr;
  }

  /**
   * The functions searches for a receiver with a given name.
   * @param receiverName The name of the receiver.
   * @return If the receiver was found, a pointer to it is returned.
   *         Otherwise, the function returns 0.
   */
  virtual ReceiverList* lookupReceiver(const std::string& receiverName)
  {
    return process.getFirstReceiver() ? process.getFirstReceiver()->lookup(name, receiverName) : nullptr;
  }

  /**
   * The function returns the name of the process.
   * @return the name of the process.
   */
  virtual const std::string& getName() const
  {
    return name;
  }

  /**
   * The function returns a pointer to the process if it has the given name.
   * @param processName The name of the process that is searched for.
   * @return If the process has the required name, a pointer to it is
   *         returned. Otherwise, the function returns 0.
   */
  virtual PlatformProcess* getProcess(const std::string& processName)
  {
    if (name == processName)
      return &process;
    else
      return nullptr;
  }

  /**
   * The function announces that the thread shall terminate.
   * It will not try to kill the thread.
   */
  void announceStop()
  {
    Thread<ProcessBase>::announceStop();
    process.trigger();
  }
};

class ProcessList;

/**
 * The class is a base class for process creators.
 */
class ProcessCreatorBase
{
private:
  static ProcessCreatorBase* first; /**< The head of the list of all process creators. */
  ProcessCreatorBase* next; /**< The next process creator in the list. */

protected:
  /**
   * The function creates a process.
   * @return A pointer to the new process.
   */
  virtual std::unique_ptr<ProcessBase> create(Logger* logger, Settings& settings) const = 0;

public:
  ProcessCreatorBase() : next(first) { first = this; }
  virtual ~ProcessCreatorBase() = default;

  friend class ProcessList;
};

/**
 * The template class instatiates creators for processes of a certain type.
 */
template <class T> class ProcessCreator : public ProcessCreatorBase
{
private:
  std::string name; /**< The name of the process that will be created. */

protected:
  /**
   * The function creates a process.
   * @return A pointer to the new process.
   */
  std::unique_ptr<ProcessBase> create(Logger* logger, Settings& settings) const { return std::make_unique<T>(name, logger, settings); }

public:
  /**
   * @param name The name of the process that will be created.
   */
  ProcessCreator(const std::string& name) : name(name) {}
};

/**
 * The class implements a list of processes.
 */
class ProcessList : public std::list<std::unique_ptr<ProcessBase>>
{
public:
  ~ProcessList()
  {
    while (!empty())
      pop_back();
  }

  /**
   * Creates a process for each process constructor and inserts them
   * into the list.
   * @param logger Pointer to Logger instance.
   * @param settings Pointer to Settings instance.
   */
  void create(Logger* logger, Settings& settings)
  {
    for (const ProcessCreatorBase* i = ProcessCreatorBase::first; i; i = i->next)
      push_back(i->create(logger, settings));
  }

  /**
   * The function announces to all processes in the list that they should stop.
   */
  void announceStop()
  {
    for (const auto& i : *this)
      i->announceStop();
  }

  /**
   * The function waits for all processes in the list to stop.
   */
  void stop()
  {
    for (const auto& i : *this)
      i->stop();
  }

  /**
   * The function starts all processes in the list.
   */
  void start()
  {
    for (const auto& i : *this)
      i->start();
  }
};

STREAMABLE(ConnectionParameter,
  STREAMABLE(ProcessConnection,,
    (std::string) sender,
    (std::string) receiver
  ),

  (std::vector<ProcessConnection>) processConnections
);

/**
 * The macro MAKE_PROCESS instatiates a process creator.
 * As a convention, it should be used in the last line of the
 * source file. For each process, MAKE_PROCESS must exactly be used
 * once.
 * @param className The type of the class that will later be instantiated
 *                 as a process.
 */
#define MAKE_PROCESS(className) ProcessCreator<ProcessFrame<className>> _create##className(#className)
