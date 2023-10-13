#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(SonarSensorData,
  ,
  (unsigned)(0) timestamp, /**< The time when the sonar measurements were taken. */
  (float)(0) leftDistanceM, /**< measurement of the left sensor (in m) */
  (float)(0) rightDistanceM /**< measurement of the right sensor (in m) */
);
