/**
 * @file MotionLogDataProvider.cpp
 * This file implements a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionLogDataProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Settings.h"

MAKE_MODULE(MotionLogDataProvider, motionInfrastructure)

CycleLocal<MotionLogDataProvider*> MotionLogDataProvider::theInstance(nullptr);

MotionLogDataProvider::MotionLogDataProvider() : frameDataComplete(false)
{
  theInstance = this;
}

MotionLogDataProvider::~MotionLogDataProvider()
{
  theInstance.reset();
}

void MotionLogDataProvider::update(GroundTruthOdometryData& groundTruthOdometryData)
{
  Pose2f odometryOffset(groundTruthOdometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionLogDataProvider:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionLogDataProvider:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionLogDataProvider:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = groundTruthOdometryData;
}

bool MotionLogDataProvider::handleMessage(InMessage& message)
{
  return *theInstance && (*theInstance)->handleMessage2(message);
}

bool MotionLogDataProvider::isFrameDataComplete()
{
  if (!*theInstance)
    return true;
  else if ((*theInstance)->frameDataComplete)
  {
    OUTPUT(idLogResponse, bin, '\0');
    (*theInstance)->frameDataComplete = false;
    return true;
  }
  else
    return false;
}

bool MotionLogDataProvider::handleMessage2(InMessage& message)
{
  switch (message.getMessageID())
  {
  case idJointAngles:
    if (handle(message) && Blackboard::getInstance().exists("FrameInfo"))
    {
      FrameInfo& frameInfo = (FrameInfo&)Blackboard::getInstance()["FrameInfo"];
      const JointAngles& jointAngles = Blackboard::get<JointAngles>();
      if (jointAngles.timestamp != frameInfo.time)
      {
        frameInfo.cycleTime = 0.012f;
        frameInfo.time = jointAngles.timestamp;
      }
    }
    return true;

  case idJointSensorData:
    if (handle(message) && Blackboard::getInstance().exists("FrameInfo"))
    {
      FrameInfo& frameInfo = (FrameInfo&)Blackboard::getInstance()["FrameInfo"];
      const JointSensorData& jointSensorData = Blackboard::get<JointSensorData>();
      if (jointSensorData.timestamp != frameInfo.time)
      {
        frameInfo.cycleTime = 0.012f;
        frameInfo.time = jointSensorData.timestamp;
      }
    }
    return true;

  case idGroundTruthOdometryData:
    if (handle(message) && Blackboard::getInstance().exists("OdometryData"))
      (OdometryData&)Blackboard::getInstance()["OdometryData"] = (OdometryData&)Blackboard::getInstance()["GroundTruthOdometryData"];
    return true;

  case idProcessFinished:
    frameDataComplete = true;
    return true;

  case idStopwatch:
  {
    const int size = message.getMessageSize();
    std::vector<unsigned char> data;
    data.resize(size);
    message.bin.read(&data[0], size);
    Global::getDebugOut().bin.write(&data[0], size);
    Global::getDebugOut().finishMessage(idStopwatch);
    return true;
  }

  default:
    return handle(message);
  }
}
