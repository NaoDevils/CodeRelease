/**
* @file CoPProvider.h
*
* Declaration of class CoPProvider, calculates zmp_acc
*
*/

#ifndef __CoPProvider_h_
#define __CoPProvider_h_

#include "Tools/Module/Module.h"
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
};


#endif //__CoPProvider_h_
