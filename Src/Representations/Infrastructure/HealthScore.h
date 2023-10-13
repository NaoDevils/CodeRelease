#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(HealthScore,,
  (float)(0) score,
  (bool)(0) top,
  (bool)(0) usable,
  (bool)(0) miserable
);
