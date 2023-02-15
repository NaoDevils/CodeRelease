/**
 * @file Modules/MotionControl/StandEngine.h
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/MessageQueue/InMessage.h"

MODULE(StandEngine,
  REQUIRES(FrameInfo),
  REQUIRES(MotionSelection),
  REQUIRES(StiffnessSettings),
  REQUIRES(JoinedIMUData),
  REQUIRES(FsrSensorData),
  REQUIRES(RobotInfo),
  REQUIRES(MotionRequest),
  PROVIDES(StandEngineOutput),
  LOADS_PARAMETERS(,
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
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource
  )
);

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
  * Default constructor & destructor.
  */
  StandEngine();
  ~StandEngine();
};
