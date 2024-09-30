/**
 * @file Process.cpp
 *
 * Implementation of class Process.
 */

#include "Process.h"
#include "Tools/Global.h"

bool MultiDebugSenderBase::terminating = false;

Process::Process(MessageQueue& debugIn, MessageQueue& debugOut, Settings& settings) : debugIn(debugIn), debugOut(debugOut), settings(settings)
{
  initialized = false;
}

bool Process::processMain()
{
  if (!initialized)
  {
    init();
    initialized = true;
  }

  handleAllMessages(debugIn);
  debugIn.clear();

  bool wait = main();

  if (Global::getDebugRequestTable().poll)
  {
    if (Global::getDebugRequestTable().pollCounter++ > 10)
    {
      Global::getDebugRequestTable().poll = false;
      OUTPUT(idDebugResponse, text, "pollingFinished");
    }
  }
  return wait;
}

void Process::handleAllMessages(MessageQueue& messageQueue)
{
  debugIn.handleAllMessages(*this);
}

Global Process::getGlobals()
{
  return {.annotationManager = &annotationManager,
      .debugOut = &debugOut.out,
      .settings = &settings,
      .debugRequestTable = &debugRequestTable,
      .debugDataTable = &debugDataTable,
      .streamHandler = &streamHandler,
      .drawingManager = &drawingManager,
      .drawingManager3D = &drawingManager3D,
      .timingManager = &timingManager,
      .blackboard = &blackboard};
}

std::string Process::getThreadName() const
{
  return threadName;
}

void Process::setThreadName(const std::string& threadName)
{
  this->threadName = threadName;
}

Logger* Process::getLogger()
{
  return this->logger;
}

void Process::setLogger(Logger* logger)
{
  this->logger = logger;
}

bool Process::handleMessage(InMessage& message)
{
  switch (message.getMessageID())
  {
  case idDebugRequest:
  {
    DebugRequest debugRequest;
    message.bin >> debugRequest;
    Global::getDebugRequestTable().addRequest(debugRequest);
    return true;
  }
  case idDebugDataChangeRequest:
    Global::getDebugDataTable().processChangeRequest(message);
    return true;
  default:
    return false;
  }
}
