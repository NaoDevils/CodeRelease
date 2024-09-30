/**
* @file Controller/LocalRobot.cpp
*
* Implementation of LocalRobot.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
*/

#include "LocalRobot.h"
#include "LogPlayer.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/AnnotationInfo.h"

LocalRobot::LocalRobot(Settings& settings)
    : RobotConsole(theDebugReceiver, theDebugSender, settings), theDebugReceiver(this, "Receiver.MessageQueue.O"), theDebugSender(this, "Sender.MessageQueue.S"), image(false),
      imageUpper(false), nextImageTimeStamp(0), imageLastTimeStampSent(0), jointLastTimeStampSent(0), updatedSignal(1), puppet(0)
{
  mode = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getMode();
  addViews();

  if (mode == SystemCall::logfileReplay)
  {
    logFile = ((ConsoleRoboCupCtrl*)RoboCupCtrl::controller)->getLogFile();
    logPlayer.open(logFile.c_str());
    handleAllMessagesByCycle(logPlayer, annotationInfos);
    logPlayer.play();
    puppet = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject("RoboCup.puppets." + robotName, SimRobotCore2::body);
    if (puppet)
      simulatedRobot.init(puppet);
  }
  else if (mode == SystemCall::simulatedRobot)
  {
    SimRobotCore2::Body* robot = (SimRobotCore2::Body*)RoboCupCtrl::application->resolveObject(RoboCupCtrl::getRobotFullName(), SimRobotCore2::body);
    ASSERT(robot);
    simulatedRobot.init(robot);
    ctrl->gameController.registerSimulatedRobot(robotName.mid(5).toInt() - 1, simulatedRobot);
  }
}

bool LocalRobot::main()
{
  if (updateSignal.tryWait())
  {
    {
      // Only one thread can access *this now.
      SYNC;

      if (mode == SystemCall::simulatedRobot)
      {
        if (jointLastTimeStampSent != jointSensorData.timestamp)
        {
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessBegin);
          debugOut.out.bin << jointSensorData;
          debugOut.out.finishMessage(idJointSensorData);
          debugOut.out.bin << fsrSensorData;
          debugOut.out.finishMessage(idFsrSensorData);
          debugOut.out.bin << inertialSensorData;
          debugOut.out.finishMessage(idInertialSensorData);
          debugOut.out.bin << sonarSensorData;
          debugOut.out.finishMessage(idSonarSensorData);
          debugOut.out.bin << odometryData;
          debugOut.out.finishMessage(idGroundTruthOdometryData);
          debugOut.out.bin << ++ping;
          debugOut.out.finishMessage(idPingpong);
          debugOut.out.bin << 'm';
          debugOut.out.finishMessage(idProcessFinished);
          jointLastTimeStampSent = jointSensorData.timestamp;
        }

        if (imageLastTimeStampSent != image.timeStamp)
        {
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessBegin);
          if (ctrl->calculateImage)
          {
            debugOut.out.bin << image;
            debugOut.out.finishMessage(idImage);
            debugOut.out.bin << imageUpper;
            debugOut.out.finishMessage(idImageUpper);
          }
          else
          {
            FrameInfo frameInfo;
            frameInfo.time = image.timeStamp;
            frameInfo.cycleTime = 1.f / (float)ctrl->calculateImageFps;
            debugOut.out.bin << frameInfo;
            debugOut.out.finishMessage(idFrameInfo);
          }
          debugOut.out.bin << cameraInfo;
          debugOut.out.finishMessage(idCameraInfo);
          debugOut.out.bin << cameraInfoUpper;
          debugOut.out.finishMessage(idCameraInfoUpper);
          debugOut.out.bin << worldState;
          debugOut.out.finishMessage(idGroundTruthWorldState);
          ctrl->gameController.writeGameInfo(debugOut.out.bin);
          debugOut.out.finishMessage(idRawGameInfo);
          int robot = robotName.mid(5).toInt() - 1;
          ctrl->gameController.writeOwnTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOwnTeamInfo);
          ctrl->gameController.writeOpponentTeamInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idOpponentTeamInfo);
          ctrl->gameController.writeRobotInfo(robot, debugOut.out.bin);
          debugOut.out.finishMessage(idRobotInfo);
          debugOut.out.bin << 'c';
          debugOut.out.finishMessage(idProcessFinished);
          imageLastTimeStampSent = image.timeStamp;
        }
      }
      else
      {
        updatedSignal.post();
      }
      theDebugSender.send(true);
    }
  }
  return true;
}

void LocalRobot::update()
{
  RobotConsole::update();

  updatedSignal.wait();

  // Only one thread can access *this now.
  {
    SYNC;

    if (mode == SystemCall::logfileReplay)
    {
      if (logAcknowledged && logPlayer.replay())
        logAcknowledged = false;
      if (puppet)
        simulatedRobot.getAndSetJointData(getJointRequest(), jointSensorData);
    }
    if (mode == SystemCall::simulatedRobot || puppet)
    {
      if (moveOp != noMove)
      {
        if (moveOp == moveBoth)
          simulatedRobot.moveRobot(movePos, moveRot * 1_deg, true);
        else if (moveOp == movePosition)
          simulatedRobot.moveRobot(movePos, Vector3f::Zero(), false);
        else if (moveOp == moveBallPosition)
          simulatedRobot.moveBall(movePos);
        else if (moveOp == kickBallVelocity)
          simulatedRobot.setBallVelocity(Vector3f(kickVelocity * std::cos(kickAngle), kickVelocity * std::sin(kickAngle), 0));
        moveOp = noMove;
      }
    }
    if (mode == SystemCall::simulatedRobot)
    {
      unsigned now = SystemCall::getCurrentSystemTime();
      if (now >= nextImageTimeStamp)
      {
        unsigned newNextImageTimeStamp = ctrl->globalNextImageTimeStamp;
        if (newNextImageTimeStamp == nextImageTimeStamp)
        {
          int imageDelay = (2000 / ctrl->calculateImageFps + 1) >> 1;
          int duration = now - ctrl->globalNextImageTimeStamp;
          ctrl->globalNextImageTimeStamp = (duration >= imageDelay ? now : ctrl->globalNextImageTimeStamp) + imageDelay;
          newNextImageTimeStamp = ctrl->globalNextImageTimeStamp;
        }
        nextImageTimeStamp = newNextImageTimeStamp;

        if (ctrl->calculateImage)
        {
          simulatedRobot.getImages(image, imageUpper, cameraInfo, cameraInfoUpper);
        }
        else
        {
          simulatedRobot.getCameraInfos(cameraInfo, cameraInfoUpper);
          image.timeStamp = now;
          imageUpper.timeStamp = now;
        }
        simulatedRobot.getRobotPose(robotPose);
        simulatedRobot.getWorldState(worldState);
      }
      else
        simulatedRobot.getRobotPose(robotPose);

      simulatedRobot.getOdometryData(robotPose, odometryData);
      simulatedRobot.getSensorData(fsrSensorData, inertialSensorData, sonarSensorData);
      simulatedRobot.getAndSetJointData(getJointRequest(), jointSensorData);
    }

    std::string statusText;
    if (mode == SystemCall::logfileReplay)
    {
      statusText = std::string("replaying ") + logFile + " ";
      if (logPlayer.currentFrameNumber != -1)
      {
        char buf[33];
        sprintf(buf, "%u", logPlayer.currentFrameNumber + 1);
        statusText += buf;
      }
      else
        statusText += "finished";
    }

    if (mode != SystemCall::logfileReplay && logPlayer.numberOfFrames != 0)
    {
      if (statusText != "")
        statusText += ", ";
      statusText += std::string("recorded ");
      char buf[33];
      sprintf(buf, "%u", logPlayer.numberOfFrames);
      statusText += buf;
    }

    if (pollingFor)
    {
      statusText += statusText != "" ? ", polling for " : "polling for ";
      statusText += pollingFor;
    }

    if (!statusText.empty())
      ((ConsoleRoboCupCtrl*)ConsoleRoboCupCtrl::controller)->printStatusText((robotName + ": " + statusText.c_str()).toUtf8().constData());
  }

  updateSignal.post();
  trigger(); // invoke a call of main()
}

bool LocalRobot::handleMessage(InMessage& message)
{
  if (message.getMessageID() == idPingpong)
  {
    unsigned timestamp;
    message.bin >> timestamp;
    if (++pong == timestamp)
      updatedSignal.post();
    return true;
  }
  return RobotConsole::handleMessage(message);
}
