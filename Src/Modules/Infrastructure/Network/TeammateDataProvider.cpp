/**
 * @file Modules/Infrastructure/TeammateDataProvider.cpp
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#include "TeammateDataProvider.h"
#include "Tools/Debugging/Annotation.h"

#include <iostream>
#include <algorithm>

void TeammateDataProvider::update(TeammateData& teammateData)
{
  // reset message budget to GC value before adding messages
  if (theGameInfo.timeLastPackageReceived == theFrameInfo.time)
    teammateData.messageBudget = theOwnTeamInfo.messageBudget;

  // in INITIAL, FINISHED and PENALTY_SHOOTOUT, message budget is ignored
  // otherwise set to starting value, see Rules 2023
  if (theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED || theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    teammateData.messageBudget = 1200;

  for (const auto& message : theTeamCommInput.messages)
  {
    TeammateReceived* teammate = nullptr;
    {
      TeammateReceived newTeammate;

      if (!newTeammate.fromTeamCommData(message))
        continue;

      if (newTeammate.teamNumber != theOwnTeamInfo.teamNumber)
        continue;

      newTeammate.updateTimestamps(theTimeOffsets);

      // Look for existing teammate
      if (newTeammate.playerNumber == theRobotInfo.number)
        teammate = &teammateData.myself;
      else if (isLatencyOkay(newTeammate))
        teammate = teammateData.getPlayer(newTeammate.playerNumber);
      else
        continue;

      // Add new teammate if it doesn't
      if (!teammate)
        teammate = &teammateData.teammates.emplace_back();

      *teammate = std::move(newTeammate);
    }

    // Count valid team messages (including own), but only in READY, SET or PLAYING
    if (theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING)
    {
      --teammateData.messageBudget;

      ++teammateData.statistic.players[teammate->playerNumber - 1];
      ++teammateData.statistic.states[theGameInfo.state];
      for (TeamCommEvents::SendReason reason : teammate->teamCommEvents.sendReasons)
        ++teammateData.statistic.events[reason];
    }

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

      if (teammate->sendTimestamp > eventmessage->sendTimestamp || (teammate->sendTimestamp == eventmessage->sendTimestamp && teammate->playerNumber < eventmessage->playerNumber))
        *eventmessage = *teammate;
    }
  }

  // Update teammate status with GameController information
  for (auto& teammate : teammateData.teammates)
    teammate.updateStatus(theOwnTeamInfo);
  teammateData.myself.updateStatus(theOwnTeamInfo);

  const auto isActiveTeammate = [](const TeammateReceived& teammate)
  {
    return teammate.status != TeammateReceived::Status::INACTIVE;
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

bool TeammateDataProvider::isLatencyOkay(const TeammateReceived& teammate)
{
  // Send timestamp may be in the future if time synchronization wasn't successful.
  // Discard package here, otherwise we keep packages in newestEventMessages forever.
  if (std::abs(static_cast<int64_t>(teammate.receiveTimestamp) - static_cast<int64_t>(teammate.sendTimestamp)) > badWifiTimeout)
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

MAKE_MODULE(TeammateDataProvider, cognitionInfrastructure)
