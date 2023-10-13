/**
 * @file TeammateData.cpp
 *
 * Representation of information received from my teammates
 *
 * @author Aaron Larisch
 */

#include <utility>
#include "TeammateData.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/Time.h"

TeamCommOutput Teammate::toTeamCommData() const
{
  TeamCommOutput tc;

  tc.data.resize(TeamCommOutput::maximumSize);
  OutCompressedBinaryMemory mem(tc.data.data());

  bool full = false;
  const auto stream = [&](const auto& data, const char* name)
  {
    if (full)
      return;

    OutCompressedBinarySize sizeStream;
    sizeStream << data;
    if (mem.getLength() + sizeStream.getSize() <= tc.data.size())
    {
      mem << data;
    }
    else
    {
      OUTPUT_WARNING("TeamCommOutput: Message too big, removed " << name);
      full = true;
    }
  };

  stream(version, "version");
  stream(playerNumber, "playerNumber");
  stream(teamNumber, "teamNumber");
  stream(sendTimestamp, "sendTimestamp");
  stream(fallen, "fallen");
  stream(timeSynchronization, "timeSynchronization");
  stream(robotPose, "robotPose");
  stream(ballModel, "ballModel");
  stream(behaviorData, "behaviorData");
  stream(teamCommEvents, "teamCommEvents");
  stream(whistle, "whistle");
  stream(speedInfo, "speedInfo");
  stream(localRobotMap, "localRobotMap");

  tc.data.resize(mem.getLength());
  tc.sendThisFrame = true;

  return tc;
}

bool TeammateReceived::fromTeamCommData(const TeamCommDataReceived& teamCommData)
{
  InCompressedBinaryMemory mem(teamCommData.data.data(), teamCommData.data.size());

  const auto stream = [&](auto& data)
  {
    if (!mem.getEof())
      mem >> data;
  };
  stream(version);

  if (version != totalVersion)
    return false;

  stream(playerNumber);
  stream(teamNumber);
  stream(sendTimestamp);
  stream(fallen);
  stream(timeSynchronization);
  stream(robotPose);
  stream(ballModel);
  stream(behaviorData);
  stream(teamCommEvents);
  stream(whistle);
  stream(speedInfo);
  stream(localRobotMap);

  if (!mem.getEof())
    OUTPUT_WARNING("TeamCommDataReceived: There is more data available than needed");

  receiveTimestamp = teamCommData.receiveTimestamp;
  remoteIp = teamCommData.remoteIp;

  return true;
}

void TeammateReceived::updateStatus(const OwnTeamInfo& ownTeamInfo)
{
  if (behaviorData.soccerState == BehaviorData::penalized || ownTeamInfo.players[playerNumber - 1].penalty != PENALTY_NONE)
    status = Status::INACTIVE;
  else if (fallen)
    status = Status::ACTIVE;
  else
    status = Status::FULLY_ACTIVE;
}

void TeammateReceived::updateTimestamps(const TimeOffsets& timeOffsets)
{
  timeOffsets.convertRemoteTimeInLocalTime(sendTimestamp, playerNumber);
  if (whistle.lastDetectionTime != 1)
    timeOffsets.convertRemoteTimeInLocalTime(whistle.lastDetectionTime, playerNumber);
  timeOffsets.convertRemoteTimeInLocalTime(ballModel.timeWhenLastSeen, playerNumber);
}

void TeammateData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeammateData", "drawingOnField");
  for (auto const& teammate : teammates)
  {
    ColorRGBA posCol;
    if (teammate.status == TeammateReceived::Status::FULLY_ACTIVE)
      posCol = ColorRGBA::green;
    else if (teammate.status == TeammateReceived::Status::ACTIVE)
      posCol = ColorRGBA::yellow;
    else
      posCol = ColorRGBA::red;

    const Vector2f& rPos = teammate.robotPose.translation;
    const float radius = 400.f; // std::max(50.f, teammate.pose.deviation);
    Vector2f dirPos = teammate.robotPose * Vector2f(radius, 0.f);

    // Circle around Player
    CIRCLE("representation:TeammateData", rPos.x(), rPos.y(), radius, 20, Drawings::solidPen, posCol, Drawings::noBrush, ColorRGBA::white);
    // Direction of the Robot
    LINE("representation:TeammateData", rPos.x(), rPos.y(), dirPos.x(), dirPos.y(), 20, Drawings::solidPen, posCol);
    // Player number
    DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y(), 100, ColorRGBA::black, teammate.playerNumber);
    // Role
    //DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y() - 150, 100,
    //         ColorRGBA::black, BehaviorData::RoleAssignment::getName(teammate.behaviorData.role));
    // Intention
    std::string intentionStr;
    switch (teammate.behaviorData.soccerState)
    {
    case BehaviorData::waiting:
      intentionStr = "default";
      break;
    case BehaviorData::positioning:
      if (teammate.behaviorData.role == BehaviorData::keeper)
        intentionStr = "keeper";
      else
        intentionStr = "defensive";
      break;
    case BehaviorData::controlBall:
      intentionStr = "kick";
      break;
    case BehaviorData::penalized:
    case BehaviorData::searchForBall:
      intentionStr = "lost";
      break;
    default:
      intentionStr = "unknown";
    }
    DRAWTEXT("representation:TeammateData", rPos.x() + 100, rPos.y() - 450, 100, ColorRGBA::black, "Intention: " << intentionStr);

    const Vector2f bPos = teammate.robotPose * teammate.ballModel.position;
    //Line from Robot to WalkTarget
    //LINE("representation:TeammateData", rPos.x(), rPos.y(), teammate.walkRequest.request.translation.x(), teammate.walkRequest.request.translation.y(), 10, Drawings::dashedPen, ColorRGBA::magenta);
    // Ball position
    CIRCLE("representation:TeammateData", bPos.x(), bPos.y(), 50, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::yellow);
  }
}

TeammateReceived* TeammateData::getPlayer(uint8_t number)
{
  return const_cast<TeammateReceived*>(std::as_const(*this).getPlayer(number));
}

const TeammateReceived* TeammateData::getPlayer(uint8_t number) const
{
  if (myself.playerNumber == number)
    return &myself;

  for (const TeammateReceived& teammate : teammates)
  {
    if (teammate.playerNumber == number)
      return &teammate;
  }

  return nullptr;
}


TeammateReceived* TeammateData::getNewestTeammate()
{
  return const_cast<TeammateReceived*>(std::as_const(*this).getNewestTeammate());
}

const TeammateReceived* TeammateData::getNewestTeammate() const
{
  const TeammateReceived* newestTeammate = nullptr;

  if (myself.playerNumber >= 0)
    newestTeammate = &myself;

  for (const TeammateReceived& teammate : teammates)
  {
    if (!newestTeammate || teammate.sendTimestamp > newestTeammate->sendTimestamp
        || (teammate.sendTimestamp == newestTeammate->sendTimestamp && teammate.playerNumber < newestTeammate->playerNumber))
      newestTeammate = &teammate;
  }

  return newestTeammate;
}

TeammateReceived* TeammateData::getNewestEventMessage(TeamCommEvents::SendReason reason)
{
  return const_cast<TeammateReceived*>(std::as_const(*this).getNewestEventMessage(reason));
}

const TeammateReceived* TeammateData::getNewestEventMessage(TeamCommEvents::SendReason reason) const
{
  for (auto& eventMessage : newestEventMessages)
  {
    if (eventMessage.reason == reason)
      return &eventMessage.message;
  }
  return nullptr;
}
