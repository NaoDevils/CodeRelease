/**
* @file Processes/Motion.cpp
* Implementation of a class that represents the process that sends commands to the robot at 50Hz.
*/

#include "Motion.h"
#include "Modules/Infrastructure/MotionLogDataProvider.h"
#include "Modules/Infrastructure/NaoProvider.h"
#include "Modules/Infrastructure/NaoProviderV6.h"
#include "Modules/MotionControl/MotionSelector.h"
#include "Modules/MotionControl/SpecialActions.h"
//#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"

Motion::Motion() :
  INIT_DEBUGGING,
  INIT_RECEIVER(CognitionToMotion),
  INIT_SENDER(MotionToCognition),
  moduleManager({ModuleBase::motionInfrastructure, ModuleBase::motionControl, ModuleBase::sensing, ModuleBase::dortmundWalkingEngine}),
  logger("Motion", 'm', Global::getSettings().naoVersion == RobotConfig::NaoVersion::V5 ? 100 : 83)
{
  theDebugReceiver.setSize(500000);
  theDebugSender.setSize(50000, 20000);

  theMotionToCognitionSender.moduleManager = theCognitionToMotionReceiver.moduleManager = &moduleManager;

  if(SystemCall::getMode() == SystemCall::physicalRobot)
    setPriority(15);
}

void Motion::init()
{
  moduleManager.load();
  BH_TRACE_INIT("Motion");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'm');

  if (Global::getSettings().naoVersion == RobotConfig::V6)
  {
    waitForFrameData = &NaoProviderV6::waitForFrameData;
    finishFrame = &NaoProviderV6::finishFrame;
  }
  else
  {
    waitForFrameData = &NaoProvider::waitForFrameData;
    finishFrame = &NaoProvider::finishFrame;
  }
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
  if(theCognitionToMotionReceiver.timeStamp != 0 && SystemCall::getTimeSince(theCognitionToMotionReceiver.timeStamp) > 500)
  {
    MotionSelector::stand();
  }

  if(MotionLogDataProvider::isFrameDataComplete())
  {
    timingManager.signalProcessStart();
    annotationManager.signalProcessStart();

    STOPWATCH_WITH_PLOT("Motion") moduleManager.execute();

    finishFrame();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

    theMotionToCognitionSender.timeStamp = SystemCall::getCurrentSystemTime();
    theMotionToCognitionSender.send();

    timingManager.signalProcessStop();

    BH_TRACE_MSG("before logger.execute");
    logger.execute();
    BH_TRACE_MSG("after logger.execute");

    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // messages were sent in this frame -> send process finished
      OUTPUT(idProcessFinished, bin, 'm');
    }
    else
      theDebugSender.removeLastMessage();

    theDebugSender.send();

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'm');
  }

  if (Blackboard::getInstance().exists("JointSensorData"))
    waitForFrameData();
  else
    SystemCall::sleep(10);

  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Motion::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    case idModuleRequest:
    {
      unsigned timeStamp;
      message.bin >> timeStamp;
      moduleManager.update(message.bin, timeStamp);
      return true;
    }
    default:
      return MotionLogDataProvider::handleMessage(message) ||
             SpecialActions::handleMessage(message) ||
             //WalkingEngine::handleMessage(message) || // TODO: some stuff with dortmund walk
             Process::handleMessage(message);
  }
}

MAKE_PROCESS(Motion);
