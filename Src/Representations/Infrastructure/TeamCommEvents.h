/**
* \file TeamCommEvents.h
* The file declares a class that containts data about all team comm events.
* Send decisions and send reasons are the output.
* \author Ingmar Schwarz
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"
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
    symmetryUpdate
  );
  ,
  (bool) sendThisFrame,
  (std::vector<SendReason>)({}) sendReasons
);
