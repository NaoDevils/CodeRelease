#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"
#include <algorithm>

MODULE(AutoCalibrator,
{ ,
  REQUIRES(FrameInfo),
  USES(WalkingInfo),
  USES(SpeedInfo),
  REQUIRES(WalkingEngineParams),
  REQUIRES(RobotDimensions),
  REQUIRES(JointSensorData),
  REQUIRES(MassCalibration),
  REQUIRES(RobotModel),
  REQUIRES(ZMPModel),
  REQUIRES(InertialSensorData),
  USES(JointRequest),
  USES(KinematicOutput),
  PROVIDES(WalkCalibration),
  LOADS_PARAMETERS(
  {,
    (float) posErrAlpha,
    (float) rotErrAlpha,
    (float) maxPosError,
    (float) maxOriError,
  }),
});

class AutoCalibrator : public AutoCalibratorBase
{
public:
  AutoCalibrator(void);
  ~AutoCalibrator(void);

private:
  void update(WalkCalibration& walkCalibration);
  void init(WalkCalibration& walkCalibration);
  void saveCalibration(WalkCalibration& walkCalibration);
  void loadCalibration(WalkCalibration& walkCalibration);
  Vector3f filteredPosErr, filteredOriErr, lastPosCorrection;
  RotationMatrix lastOriCorrection;

  Vector3f desiredPositionOffset, desiredRotationOffset;
  unsigned timeCalibrationStart;
  bool calibrate;
  bool calibrationLoaded;
  bool calibrationOK;
  unsigned timeStampCalibrationOK;
};
