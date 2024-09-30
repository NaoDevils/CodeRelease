#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(USBSettings,,
  (unsigned)(0) updateTimestamp,
  (std::string)("") wifiSSID,
  (std::string)("") wifiPassword,
  (std::string)("") ip,
  (uint8_t)(0) robotNumber,
  (uint8_t)(0) teamNumber,
  (uint16_t)(0) teamPort
);
