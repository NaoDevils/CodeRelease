/**
 * @file KickEngine.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "KickEngineData.h"
#include "KickEngineParameters.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"

MODULE(KickEngine,
{,
  USES(JointRequest),
  REQUIRES(FrameInfo),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointCalibration),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSelection),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(InertialSensorData),
  PROVIDES(KickEngineOutput),
});

class KickEngine : public KickEngineBase
{
private:
  KickEngineData data;
  bool compensate = false;
  bool compensated = false;
  unsigned timeSinceLastPhase = 0;

  std::vector<KickEngineParameters> params;

public:
  KickEngine();

  void update(KickEngineOutput& kickEngineOutput);
};
