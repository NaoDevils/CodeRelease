/**
* @file Processes/Cognition.cpp
* Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
*/

#include "Tools/Math/Eigen.h" // include first to avoid conflicts between Cabsl defines and some clang headers
#include "Cognition.h" // include second header conflicts on Windows
#include "Modules/Configuration/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CameraProviderV6.h"
#include "Modules/Infrastructure/CameraProvider.h"
#include "Modules/Infrastructure/CognitionLogDataProvider.h"
#include "Modules/Infrastructure/TeammateDataProvider.h"
#include "Modules/Infrastructure/MocapDataProvider.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Tools/Settings.h"

Cognition::Cognition() :
	INIT_DEBUGGING,
	INIT_RECEIVER(MotionToCognition),
	INIT_SENDER(CognitionToMotion),
	INIT_TEAM_COMM,
#ifdef USE_MOCAP
	INIT_MOCAP_COMM,
#endif // USE_MOCAP
	moduleManager({ ModuleBase::cognitionInfrastructure, ModuleBase::perception, ModuleBase::pathPlanning, ModuleBase::modeling, ModuleBase::behaviorControl }),
  logger("Cognition",'c', 30)
{
	theDebugSender.setSize(5200000, 100000);
	theDebugReceiver.setSize(2800000);
	theCognitionToMotionSender.moduleManager = theMotionToCognitionReceiver.moduleManager = &moduleManager;
}

void Cognition::init()
{
	Global::theTeamOut = &theTeamSender;
  Global::theNTP = &theNTP;
	START_TEAM_COMM;
#ifdef USE_MOCAP
	START_MOCAP_COMM;
#endif // USE_MOCAP
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
	// read from team comm udp socket
	RECEIVE_TEAM_COMM;

	// Read mocap data from mocap comm udp socket
#ifdef USE_MOCAP
	RECEIVE_MOCAP_COMM;
#endif // USE_MOCAP

	bool isFrameDataComplete = false;
	if (Global::getSettings().naoVersion == RobotConfig::V6) 
	  isFrameDataComplete = CameraProviderV6::isFrameDataComplete();
	else 
	  isFrameDataComplete = CameraProvider::isFrameDataComplete();

	if (CognitionLogDataProvider::isFrameDataComplete() && isFrameDataComplete)
	{
		timingManager.signalProcessStart();
		annotationManager.signalProcessStart();

		BH_TRACE_MSG("before TeammateDataProvider");
		TeammateDataProvider::handleMessages(theTeamReceiver);

#ifdef USE_MOCAP
		MocapDataProvider::handleMessages(theMocapReceiver);
#endif // USE_MOCAP

		// Reset coordinate system for debug field drawing
		DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)
		ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

		STOPWATCH_WITH_PLOT("Cognition") moduleManager.execute();

		DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
		DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
		DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

		theCognitionToMotionSender.timeStamp = SystemCall::getCurrentSystemTime();
		BH_TRACE_MSG("before cognition2Motion.send");
		theCognitionToMotionSender.send();
		BH_TRACE_MSG("after cognition2Motion.send");

		BH_TRACE_MSG("before SEND_TEAM_COMM");
		if (!theTeamSender.isEmpty())
		{
			if (Blackboard::getInstance().exists("TeammateData") &&
				static_cast<const TeammateData&>(Blackboard::getInstance()["TeammateData"]).sendThisFrame)
			{
				SEND_TEAM_COMM;
			}
			theTeamSender.clear(); // team messages are purged even when not sent.
		}
		BH_TRACE_MSG("after SEND_TEAM_COMM");

		timingManager.signalProcessStop();

		BH_TRACE_MSG("before logger.execute");
		logger.execute();
		BH_TRACE_MSG("after logger.execute");

		DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

		DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
		annotationManager.clear();

		if (theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
		{
			// messages were sent in this frame -> send process finished
			// Dortmund : only one cognition frame for both cams
			/*if(Blackboard::getInstance().exists("CameraInfo") &&
			static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::lower)
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
		theDebugSender.send();

		// Prepare next frame
		numberOfMessages = theDebugSender.getNumberOfMessages();
		OUTPUT(idProcessBegin, bin, 'c');
	}
	else if (Global::getDebugRequestTable().poll)
		--Global::getDebugRequestTable().pollCounter;

	if (Blackboard::getInstance().exists("Image"))
	{
		if (SystemCall::getMode() == SystemCall::physicalRobot)
			setPriority(10);
		SystemCall::sleep(1);
		BH_TRACE_MSG("before waitForFrameData");
		if (Global::getSettings().naoVersion == RobotConfig::V6) 
			CameraProviderV6::waitForFrameData();
		else 
			CameraProvider::waitForFrameData();

		if (SystemCall::getMode() == SystemCall::physicalRobot)
			setPriority(0);
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
		return CognitionLogDataProvider::handleMessage(message) ||
			CognitionConfigurationDataProvider::handleMessage(message) ||
			Process::handleMessage(message);
	}
	BH_TRACE_MSG("after Cognition:handleMessage");
}

MAKE_PROCESS(Cognition);
