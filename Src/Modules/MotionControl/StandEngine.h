/**
 * @file Modules/MotionControl/StandEngine.h
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Module.h"

MODULE(StandEngine,
{ ,
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  REQUIRES(MotionSelection),
  REQUIRES(StiffnessSettings),
  REQUIRES(InertialSensorData),
  REQUIRES(FsrSensorData),
  REQUIRES(RobotInfo),
  REQUIRES(MotionRequest),
  PROVIDES(StandEngineOutput),
  LOADS_PARAMETERS(
  { ,
    (Angle) targetAngle,
    (Angle) targetAngleDeviation,
    (Angle) targetAngleDeviationNoStiffness,
    (Angle) targetAngleOffsetStep,
    (Angle) targetAngleOffsetMax,
    (float[3]) angleYpid,
    (int) transitionStart,
    (int) transitionTime,
    (Angle) leaveTransitionSpeed,
    (Angle[Joints::numOfJoints]) jointAngles,
    (int[Joints::numOfJoints]) defaultStiffnesses,
    (int[Joints::numOfJoints]) lowStiffnesses,
  }),
});

class StandEngine : public StandEngineBase
{
private:
  void update(StandEngineOutput& standEngineOutput);

  Angle angleSum = 0_deg;
  Angle angleOldError = 0_deg;
  unsigned int stablePositionTimestamp = 0;
  Angle targetAngleOffset = 0_deg;

  Angle pitchOffset = 0_deg;

  float fsrMin = INFINITY;
  float fsrMax = -INFINITY;

public:
  /*
  * Default constructor.
  */
  StandEngine();
};
