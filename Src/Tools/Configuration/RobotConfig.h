#pragma once

#include <map>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

STREAMABLE(RobotConfig,
  ENUM(NaoVersion,
    V5,
    V6
  );

  ,
  (std::string) name,
  (std::string) headId,
  (std::string) bodyId,
  (std::string) lan,
  (std::string) wlan,
  (std::string) id,
  (NaoVersion) naoVersion
);
