#pragma once

#include "SensorData/FsrSensorData.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

#include <array>

STREAMABLE_WITH_BASE(FsrModelData, FsrSensorData,
  ,
  (float)(0.f) total, /**< Total mass pressing both foots (in kg) */
  (bool)(false) modelDataReady,
  (bool)(true) leftFootOnGround,
  (bool)(true) rightFootOnGround
);
