/**
* @file Processes/Motion.cpp
* Implementation of a class that represents the process that sends commands to the robot at 50Hz.
*/

#include "Motion.h"
#include "Modules/Infrastructure/MotionLogDataProvider.h"
#include "Modules/Infrastructure/NaoProviderV6.h"
#include "Modules/MotionControl/MotionSelector.h"
#include "Modules/MotionControl/KeyFrameEngine/KeyFrameEngine.h"
#include "Tools/Build.h"
#include "Tools/Settings.h"
//#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"

Motion::Motion(Settings& settings)
    : INIT_SUPERTHREAD_DEBUGGING("motion.cfg"), INIT_RECEIVER(CognitionToMotion), INIT_SENDER(MotionToCognition),
      moduleManager({ModuleBase::motionInfrastructure, ModuleBase::motionControl, ModuleBase::sensing, ModuleBase::dortmundWalkingEngine}, this)
{
  theDebugReceiver.setSize(1000000);
  theDebugSender.setSize(5200000, 100000);

  theMotionToCognitionSender.moduleManager = theCognitionToMotionReceiver.moduleManager = &moduleManager;

  timingManager.setThreadTime(false);

  if (SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(15);
}

void Motion::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Motion");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'm');

  ASSERT(Global::getSettings().naoVersion == RobotConfig::V6);
  waitForFrameData = &NaoProviderV6::waitForFrameData;
  finishFrame = &NaoProviderV6::finishFrame;
}

void Motion::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Motion::main()
{
  MotionSelector::move();
  // there has been no new package from Cognition in more than 500ms and thus we let the robot stand.
  if (theCognitionToMotionReceiver.timeStamp != 0 && SystemCall::getTimeSince(theCognitionToMotionReceiver.timeStamp) > 500)
  {
    MotionSelector::stand();
  }

  // Crash robot and dump log if cognition hangs for more than 15s
  if constexpr (Build::targetRobot())
    ASSERT(theCognitionToMotionReceiver.timeStamp == 0 || SystemCall::getTimeSince(theCognitionToMotionReceiver.timeStamp) < 15000);

  if (MotionLogDataProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();
    STOPWATCH_WITH_PLOT("Motion:preprocessing")
    {
      annotationManager.signalProcessStart();

      beforeRun();
    }

    {
      Stopwatch s1(MONOTONIC_WITH_PLOT("Motion:monotonic"));
      Stopwatch s2(WITH_PLOT("Motion"));
      moduleManager.execute();
    }

    STOPWATCH_WITH_PLOT("Motion:postprocessing")
    {
      finishFrame();

      afterRun();

      observeFunction("moveMessages",
          [&]
          {
            moveMessages(theDebugSender);
          });

      DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
      DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
      DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

      theMotionToCognitionSender.timeStamp = SystemCall::getCurrentSystemTime();
      observeFunction("motion2cognition",
          [&]
          {
            theMotionToCognitionSender.send();
          });
    }

    timingManager.signalProcessStop();

    BH_TRACE_MSG("before logger.execute");
    observeFunction("logger",
        [&]
        {
          getLogger()->execute('m');
        });
    BH_TRACE_MSG("after logger.execute");

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if (theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    else
      theDebugSender.removeLastMessage();

    observeFunction("debugSender",
        [&]
        {
          theDebugSender.send();
        });

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'm');
  }

  if (Blackboard::getInstance().exists("JointSensorData"))
    waitForFrameData();
  else
    SystemCall::sleep(12);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Motion::handleMessage(InMessage& message)
{
  switch (message.getMessageID())
  {
  case idModuleRequest:
  {
    unsigned timeStamp;
    message.bin >> timeStamp;
    moduleManager.update(message.bin, timeStamp);
    return true;
  }
  case idPingpong:
  {
    unsigned ts;
    message.bin >> ts;
    OUTPUT(idPingpong, bin, ts);
    return true;
  }
  default:
    return MotionLogDataProvider::handleMessage(message) || KeyFrameEngine::handleMessage(message) ||
        //WalkingEngine::handleMessage(message) || // TODO: some stuff with dortmund walk
        SuperThread::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);
