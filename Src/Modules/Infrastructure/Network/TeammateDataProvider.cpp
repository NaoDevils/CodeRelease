/**
 * @file Modules/Infrastructure/TeammateDataProvider.cpp
 * This file implements a temporary wrapper from the old to the new TeammateData structure.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "TeammateDataProvider.h"
#include "Tools/Debugging/Annotation.h"

#include <iostream>
#include <algorithm>

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * member.
 */
#define UNPACK(representation, member)                        \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed;             \
  teammate.member = static_cast<representation>(the##representation##Compressed);

void TeammateDataProvider::update(TeammateData& teammateData)
{
  // reset message budget to GC value before adding messages
  if (theGameInfo.timeLastPackageReceived == theFrameInfo.time)
    teammateData.messageBudget = theOwnTeamInfo.messageBudget;
  // in INITIAL, FINISHED and PENALTY_SHOOTOUT, message budget is ignored
  // otherwise set to starting value, see Rules 2022
  if (theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    teammateData.messageBudget = (theGameInfo.competitionType == COMPETITION_TYPE_7V7) ? 1680 : 1200;

  for (auto& message : theTeamCommInput.messages)
  {
    Teammate* teammate = nullptr;
    if (message.playerNum == theRobotInfo.number)
      teammate = &teammateData.myself;
    else if (isLatencyOkay(message))
      teammate = &getTeammate(teammateData, message);
    else
      continue;

    // Count valid team messages (including own), but only in READY, SET or PLAYING
    if (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING)
    {
      --teammateData.messageBudget;
      ++messageCounts[message.playerNum];
    }

    fillTeammate(message, *teammate);

    // update newestEventMessages
    for (const auto reason : teammate->teamCommEvents.sendReasons)
    {
      Teammate* eventmessage = teammateData.getNewestEventMessage(reason);
      if (!eventmessage)
      {
        auto& teammateevent = teammateData.newestEventMessages.emplace_back();
        teammateevent.reason = reason;
        eventmessage = &teammateevent.message;
      }

      if (teammate->timeWhenSent > eventmessage->timeWhenSent || (teammate->timeWhenSent == eventmessage->timeWhenSent && teammate->number < eventmessage->number))
        *eventmessage = *teammate;
    }
  }

  // Overwrite teammate status with GameController information
  for (auto& teammate : teammateData.teammates)
  {
    if (teammate.behaviorData.soccerState == BehaviorData::penalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
      teammate.status = Teammate::INACTIVE;
    else if (!teammate.isUpright)
      teammate.status = Teammate::ACTIVE;
    else
      teammate.status = Teammate::FULLY_ACTIVE;
  }

  const auto isActiveTeammate = [](const Teammate& teammate)
  {
    return teammate.status != Teammate::INACTIVE;
  };
  teammateData.numberOfActiveTeammates = static_cast<int>(std::count_if(teammateData.teammates.begin(), teammateData.teammates.end(), isActiveTeammate));

  // check wlanQuality
  if (wlanQuality > 0.7f)
    teammateData.wlanOK = true;
  else if (wlanQuality < 0.5f)
    teammateData.wlanOK = false;

  // if no communication is possible due to package limit, behave as if no wifi is available
  if (teammateData.messageBudget <= 5)
    teammateData.wlanOK = false;

  if (teammateData.teammates.size() != lastNoOfTeammates)
    ANNOTATION("TeammateData", "Number of Teammates changed to " << teammateData.teammates.size());
  lastNoOfTeammates = teammateData.teammates.size();

  const int16_t totalSecsRemaining = theGameInfo.secsRemaining + theGameInfo.firstHalf * 600;
  teammateData.messageBudgetFactor = std::min(1.f, static_cast<float>(teammateData.messageBudget) / totalSecsRemaining);
}

bool TeammateDataProvider::isLatencyOkay(const TeamCommData& message)
{
  unsigned int sendTimestamp = message.getNDHeader().sendTimestamp;
  theTimeOffsets.convertRemoteTimeInLocalTime(sendTimestamp, message.playerNum);

  if (std::abs(static_cast<int64_t>(theFrameInfo.time) - static_cast<int64_t>(sendTimestamp)) > badWifiTimeout)
  {
    wlanQuality = std::max(0.f, wlanQuality - 0.2f);
    return false;
  }
  else
  {
    wlanQuality = std::min(1.f, wlanQuality + 0.1f);
    return true;
  }
}

Teammate& TeammateDataProvider::getTeammate(TeammateData& teammateData, const TeamCommData& message)
{
  // Try to find the robot in the current list of robots:
  for (auto& teammate : teammateData.teammates)
  {
    if (teammate.number == message.playerNum)
      return teammate;
  }

  // This seems to be a new robot that is not part of the list yet:
  teammateData.teammates.emplace_back();
  Teammate& teammate = teammateData.teammates.back();
  return teammate;
}


void TeammateDataProvider::fillTeammate(InMessage& message, Teammate& teammate) const
{
  switch (message.getMessageID())
  {
  case idRobotPose:
  {
    UNPACK(RobotPose, pose);
    break;
  }
  case idWhistleDortmund:
  {
    message.bin >> teammate.whistle;
    theTimeOffsets.convertRemoteTimeInLocalTime(teammate.whistle.lastDetectionTime, teammate.number);
    break;
  }
  case idBallModel:
  {
    BallModel lastBallModel = teammate.ball;
    UNPACK(BallModel, ball);
    theTimeOffsets.convertRemoteTimeInLocalTime(teammate.ball.timeWhenLastSeen, teammate.number);
    theTimeOffsets.convertRemoteTimeInLocalTime(teammate.ball.timeWhenDisappeared, teammate.number);
    if (!teammate.isNDevilsPlayer && (teammate.ball.timeWhenLastSeen - lastBallModel.timeWhenLastSeen) < 16)
      teammate.ball.lastPerception = lastBallModel.lastPerception;
    break;
  }
  case idRobotsPercept:
  {
    UNPACK(RobotsPercept, robotsPercept);
    break;
  }
  case idRobotsPerceptUpper:
  {
    UNPACK(RobotsPerceptUpper, robotsPerceptUpper);
    break;
  }
  case idRobotMap:
  {
    UNPACK(RobotMap, robotMap);
    break;
  }
  case idLocalRobotMap:
  {
    UNPACK(LocalRobotMap, localRobotMap);
    break;
  }
  case idSideConfidence:
    message.bin >> teammate.sideConfidence;
    break;
  case idBehaviorData:
    message.bin >> teammate.behaviorData;
    break;
  case idSimpleRobotsDistributed:
    message.bin >> teammate.simpleRobotsDistributed;
    break;
  case idMotionRequest:
  {
    UNPACK(WalkRequest, walkRequest);
    break;
  }
  case idSpeedInfo:
  {
    UNPACK(SpeedInfo, speedInfo);
    break;
  }
  case idTeamCommEvents:
    message.bin >> teammate.teamCommEvents;
    break;
  }
}

// Only sets upright and pose information, since ball information is in ball model.
// Could be merged in the future.
void TeammateDataProvider::fillTeammate(const TeamCommData& message, Teammate& teammate) const
{
  teammate.timeWhenLastPacketReceived = message.receiveTimestamp;
  teammate.timeWhenSent = message.getNDHeader().sendTimestamp;
  theTimeOffsets.convertRemoteTimeInLocalTime(teammate.timeWhenSent, message.playerNum);
  teammate.isNDevilsPlayer = true;
  teammate.number = message.playerNum;

  teammate.isUpright = !message.fallen;
  teammate.pose.translation.x() = message.pose[0];
  teammate.pose.translation.y() = message.pose[1];
  teammate.pose.rotation = Angle::normalize(message.pose[2]);

  MessageQueue queue;
  message.fillMessageQueue(queue);
  queue.handleAllMessages(
      [&](InMessage& message)
      {
        return fillTeammate(message, teammate);
      });
}

MAKE_MODULE(TeammateDataProvider, cognitionInfrastructure)
