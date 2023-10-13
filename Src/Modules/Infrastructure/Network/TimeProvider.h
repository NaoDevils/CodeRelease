/**
 * @file Modules/Infrastructure/TimeProvider.h
 * This modules provides an NTP-like time synchronization mechanism.
 * @author <a href="mailto:aaron.larisch@udo.edu">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeamCommData.h"
#include "Representations/Infrastructure/Time.h"
#include "Representations/Infrastructure/TeamCommSenderOutput.h"
#include <unordered_map>
#include <array>

MODULE(TimeProvider,
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TeamCommInput),
  REQUIRES(TimeOffsets),
  USES(TeamCommSenderOutput),
  HAS_PREEXECUTION,
  PROVIDES(TimeOffsets),
  PROVIDES(TimeSynchronization),
  LOADS_PARAMETERS(,
    (int)(50) requestRttThreshold
  )
);

class TimeProvider : public TimeProviderBase
{
private:
  STREAMABLE(Roundtrip,
    ,
    (TimeSynchronization::Measurement) request,
    (TimeSynchronization::Measurement) response
  );

  std::unordered_map<unsigned char, Roundtrip> roundtrips;
  std::unordered_map<unsigned char, TimeSynchronization::Measurement> receivedRequests;

  std::array<unsigned char, MAX_NUM_PLAYERS> answered{false};

public:
  void execute(tf::Subflow&);
  void update(TimeOffsets& timeOffsets);
  void update(TimeSynchronization& timeSynchronization);
};
