/**
* \file TeamCommEvents.h
* The file declares a class that containts data about all team comm events.
* Send decisions and send reasons are the output.
* \author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Compressed.h"
#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"

/**
* \class TeamCommEvents
* A class that containts data about team comm events.
*/
STREAMABLE(TeamCommEvents,
  ENUM(SendReason,
    newRolesAssigned,
    playerMoved,
    goalDetected,
    kickOffFinished,
    ballMoved,
    ballchaserFallDown,
    symmetryLost,
    symmetryUpdate,
    newBallchaser,
    whistleDetected,
    timeResponse,
    refereeGestureDetected,
    uprightAgain
  );

  using SendReasonVectorCompressed = EnumVectorCompressed<SendReason COMMA SendReason::numOfSendReasons>;
  ,
  (bool) sendThisFrame,
  (std::vector<SendReason>) sendReasons
);

STREAMABLE(TeamCommEventsCompressed,
  // Increase version number whenever something changes!
  static constexpr unsigned char version = 3;

  TeamCommEventsCompressed() = default;
  explicit TeamCommEventsCompressed(const TeamCommEvents& teamCommEvents) : sendReasons(teamCommEvents.sendReasons) {};
  ,
  ((TeamCommEvents) SendReasonVectorCompressed) sendReasons
);
