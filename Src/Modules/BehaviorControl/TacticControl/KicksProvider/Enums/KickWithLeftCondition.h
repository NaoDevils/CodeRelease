#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

namespace KickInfos
{
  ENUM(KickWithLeftCondition,
    onLeftSide,
    onRightSide,
    leftFootIsClosest,
    rightFootIsClosest
  );
}
