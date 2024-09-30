/**
* @file CoPProvider.h
*
* Declaration of class CoPProvider, calculates ZMP_RCS
*
*/

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Module/Module.h"
#include "Tools/RobotParts/FootShape.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/FsrModelData.h"


MODULE(CoPProvider,
  REQUIRES(FsrSensorData),
  REQUIRES(FsrModelData),
  REQUIRES(RobotModel),
  USES(WalkingInfo),
  PROVIDES(ZMPModel),
  LOADS_PARAMETERS(,
    (bool)(false) useFsrModel
  )
);

class CoPProvider : public CoPProviderBase
{
public:
private:
  void update(ZMPModel& zmpModel);
};
