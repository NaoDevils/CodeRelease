/**
* @file Processes/Cognition.cpp
* Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
*/

#include "Tools/Math/Eigen.h" // include first to avoid conflicts between Cabsl defines and some clang headers
#include "Cognition.h" // include second header conflicts on Windows
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CameraProviderV6.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Tools/Settings.h"

Cognition::Cognition()
    : INIT_SUPERTHREAD_DEBUGGING("cognition.cfg"), INIT_RECEIVER(MotionToCognition), INIT_SENDER(CognitionToMotion),
      moduleManager({ModuleBase::cognitionInfrastructure, ModuleBase::perception, ModuleBase::pathPlanning, ModuleBase::modeling, ModuleBase::behaviorControl}, this)
{
  theDebugSender.setSize(5200000, 100000);
  theDebugReceiver.setSize(2800000);
  theCognitionToMotionSender.moduleManager = theMotionToCognitionReceiver.moduleManager = &moduleManager;

  timingManager.setThreadTime(false);

  if (SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(10);
}

void Cognition::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Cognition");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'c');
}

void Cognition::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Cognition::main()
{
  bool isFrameDataComplete = false;
  ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
  isFrameDataComplete = CameraProviderV6::isFrameDataComplete();

  if (CognitionLogDataProvider::isFrameDataComplete() && isFrameDataComplete)
  {
    timingManager.signalProcessStart();
    STOPWATCH_WITH_PLOT("Cognition:preprocessing")
    {
      annotationManager.signalProcessStart();

      // Reset coordinate system for debug field drawing
      DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
      ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

      beforeRun();
    }

    STOPWATCH_MONOTONIC_WITH_PLOT("Cognition:monotonic") STOPWATCH_WITH_PLOT("Cognition") moduleManager.execute();

    STOPWATCH_WITH_PLOT("Cognition:postprocessing")
    {
      afterRun();

      observeFunction("moveMessages",
          [&]
          {
            moveMessages(theDebugSender);
          });

      DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
      DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
      DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

      theCognitionToMotionSender.timeStamp = SystemCall::getCurrentSystemTime();
      BH_TRACE_MSG("before cognition2Motion.send");
      observeFunction("cognition2motion",
          [&]
          {
            theCognitionToMotionSender.send();
          });
      BH_TRACE_MSG("after cognition2Motion.send");
    }

    timingManager.signalProcessStop();

    BH_TRACE_MSG("before logger.execute");
    observeFunction("logger",
        [&]
        {
          getLogger()->execute('c');
        });
    BH_TRACE_MSG("after logger.execute");

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if (theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      // Dortmund : only one cognition frame for both cams
      /*if(Blackboard::getInstance().exists("CameraInfo") &&
      Blackboard::get<CameraInfo>().camera == CameraInfo::lower)
      { // lower camera -> process called 'd'
      theDebugSender.patchMessage(numberOfMessages, 0, 'd');
      OUTPUT(idProcessFinished, bin, 'd');
      }
      else*/
      OUTPUT(idProcessFinished, bin, 'c');
    }
    else
      theDebugSender.removeLastMessage();

    BH_TRACE_MSG("theDebugSender.send()");

    observeFunction("debugSender",
        [&]
        {
          theDebugSender.send();
        });

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'c');
  }
  else if (Global::getDebugRequestTable().poll)
    --Global::getDebugRequestTable().pollCounter;

  if (Blackboard::getInstance().exists("Image"))
  {
    BH_TRACE_MSG("before waitForFrameData");
    ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
    CameraProviderV6::waitForFrameData();
  }
  else
    SystemCall::sleep(33);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Cognition::handleMessage(InMessage& message)
{
  BH_TRACE_MSG("before Cognition:handleMessage");
  switch (message.getMessageID())
  {
  case idModuleRequest:
  {
    unsigned timeStamp;
    message.bin >> timeStamp;
    moduleManager.update(message.bin, timeStamp);
    return true;
  }
  default:
    return CognitionLogDataProvider::handleMessage(message) || CognitionConfigurationDataProvider::handleMessage(message) || SuperThread::handleMessage(message);
  }
  BH_TRACE_MSG("after Cognition:handleMessage");
}

MAKE_PROCESS(Cognition);
