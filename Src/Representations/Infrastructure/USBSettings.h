#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(USBSettings,,
  (unsigned)(0) updateTimestamp,
  (std::string)("") wifiSSID,
  (std::string)("") wifiPassword,
  (std::string)("") ip,
  (int)(0) robotNumber,
  (int)(0) teamNumber,
  (int)(0) teamPort
);
