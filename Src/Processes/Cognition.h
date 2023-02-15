/**
 * @file Processes/Cognition.h
 * Declaration of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#pragma once

#include "Tools/ProcessFramework/SubThread.h"
#include "Tools/ProcessFramework/Process.h"
#include "Tools/Module/ModulePackage.h"
#include "Tools/Module/Logger.h"

/**
 * @class Cognition
 * A class that represents a process that receives data from the robot at about 30 Hz.
 */
class Cognition : public SuperThread
{
private:
  DEBUGGING;
  RECEIVER(MotionToCognition);
  SENDER(CognitionToMotion);
  int numberOfMessages;
  ModuleManager moduleManager; /**< The solution manager handles the execution of modules. */

public:
  Cognition();

  /**
   * The method is called from the framework once in every frame.
   */
  virtual bool main();

  /**
   * The method is called directly before the first call of main().
   */
  virtual void init();

  /**
   * The method is called when the process is terminated.
   */
  virtual void terminate();

  /**
   * The function handles incoming debug messages.
   * @param message the message to handle.
   * @return Has the message been handled?
   */
  virtual bool handleMessage(InMessage& message);
};
