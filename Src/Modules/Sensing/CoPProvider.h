/**
* @file CoPProvider.h
*
* Declaration of class CoPProvider, calculates ZMP_RCS
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RobotParts/FootShape.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"


MODULE(CoPProvider,
  REQUIRES(FsrSensorData),
  REQUIRES(RobotModel),
  USES(WalkingInfo),
  PROVIDES(ZMPModel)
);

class CoPProvider : public CoPProviderBase
{

public:
private:
  void update(ZMPModel& zmpModel);

  void drawFoot(bool left, const Pose2f& baseInImage);
};
