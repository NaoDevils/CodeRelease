#pragma once

#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(SonarConfiguration,
,
  (float)(0) minDistanceMm,       /* minimum detection range of the sonar in mm */
  (float)(600) maxDistanceMm      /* maximum detection range of the sonar in mm */
);
