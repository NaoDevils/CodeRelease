
#include "TimeProvider.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/BHAssert.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Representations/Infrastructure/TeammateData.h"
#include <numeric>

MAKE_MODULE(TimeProvider, cognitionInfrastructure);

void TimeProvider::execute(tf::Subflow&)
{
  // clear received requests after sending
  if (theTeamCommSenderOutput.dataSent)
    receivedRequests.clear();

  for (const auto& message : theTeamCommInput.messages)
  {
    TeammateReceived teammate;
    if (!teammate.fromTeamCommData(message))
      continue;

    if (teammate.teamNumber != theOwnTeamInfo.teamNumber)
      continue;

    const TimeSynchronization& ts = teammate.timeSynchronization;

    // answer on request or never answered before
    if ((ts.requestFrom & (1 << (theRobotInfo.number - 1))) > 0 || !answered[teammate.playerNumber - 1])
    {
      auto& request = receivedRequests[teammate.playerNumber];
      request.sent = teammate.sendTimestamp;
      request.received = message.receiveTimestamp;

      answered[teammate.playerNumber - 1] = true;
    }

    // collect roundtrips
    for (const auto& request : ts.receivedRequests)
    {
      if (request.player == theRobotInfo.number)
      {
        auto& roundtrip = roundtrips[teammate.playerNumber];
        roundtrip.request = request;
        roundtrip.response.sent = teammate.sendTimestamp;
        roundtrip.response.received = message.receiveTimestamp;
      }
    }
  }
}

void TimeProvider::update(TimeOffsets& timeOffsets)
{
  for (const auto& [playerNum, roundtrip] : roundtrips)
  {
    const int reqSent = static_cast<int>(roundtrip.request.sent);
    const int reqRcvd = static_cast<int>(roundtrip.request.received);
    const int resSent = static_cast<int>(roundtrip.response.sent);
    const int resRcvd = static_cast<int>(roundtrip.response.received);

    const int rtt = (resRcvd - reqSent) - (resSent - reqRcvd);
    const int offset = ((reqRcvd - reqSent) + (resSent - resRcvd)) / 2;

    int& bestRTT = timeOffsets.bestRTT[playerNum - 1];
    int& bestOffset = timeOffsets.bestOffset[playerNum - 1];

    if (rtt < bestRTT || std::abs(bestOffset - offset) > bestRTT)
    {
      bestRTT = rtt;
      bestOffset = offset;
    }
  }
  roundtrips.clear();
}

void TimeProvider::update(TimeSynchronization& timeSynchronization)
{
  timeSynchronization.receivedRequests.clear();
  for (const auto& [player, request] : receivedRequests)
  {
    auto& tsRequest = timeSynchronization.receivedRequests.emplace_back();
    tsRequest.sent = request.sent;
    tsRequest.received = request.received;
    tsRequest.player = player;
  }

  timeSynchronization.requestFrom = 0;
  for (unsigned char player = 0; player < MAX_NUM_PLAYERS; player++)
  {
    if (player + 1 == theRobotInfo.number)
      continue;
    if (theTimeOffsets.bestRTT[player] <= requestRttThreshold)
      continue;
    timeSynchronization.requestFrom |= 1 << player;
  }
}
