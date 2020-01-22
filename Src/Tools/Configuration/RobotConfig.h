#pragma once

#include <map>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

STREAMABLE(RobotConfig,
{
  ENUM(NaoVersion,
  {,
    V5,
    V6,
  });

  std::string wlan;
  std::string lan;
  void initNetwork() { wlan = "10.0.12." + id; lan = "192.168.101." + id; }
  ,
  (std::string) name,
  (std::string) headId,
  (std::string) bodyId,
  (std::string) id,
  (NaoVersion) naoVersion,
});
