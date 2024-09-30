#pragma once

#include <map>

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"

#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/Sensing/FallDownState.h"

#include "Tools/RingBufferWithSum.h"

STREAMABLE(FsrStatus,,
  (std::array<bool,FsrSensorData::numOfFsrSensorPositions>) sensorStatusLeft,
  (std::array<bool,FsrSensorData::numOfFsrSensorPositions>) sensorStatusRight,
  (bool)(true) usableLeft,
  (bool)(true) usableRight
);

STREAMABLE(IMUStatus,,
  (std::array<bool,3>) accStatus,
  (std::array<bool,3>) gyroStatus,
  (std::array<bool,2>) angleStatus
);

STREAMABLE(JointStatus,,
  (bool)(true) usableLegs,
  (bool)(true) usableArms
);

STREAMABLE(FrameRateStatus,,
  (float)(0) motionFrameRate,
  (float)(0) cognitionFrameRate
);

STREAMABLE(WalkingStatus,,
  (std::array<int,4>) walkingErrors,
  (float)(0) stumble,
  (bool)(0) stumblingForward,
  (bool)(0) stumblingBackward,
  (bool)(0) stumblingLeft,
  (bool)(0) stumblingRight,
  (float)(0) kinematicStumble,
  (float)(0) angleStumble,
  (float)(0) fsrStumble,
  (unsigned)(0) averageStepTime,
  (std::array<float,4>) decreaseIncreaseFactors,
  (float)(0) maxSpeedForward,
  (float)(1) speedFactorForward,
  (float)(0) maxSpeedBackward,
  (float)(1) speedFactorBackward,
  (float)(0) maxSpeedLeft,
  (float)(1) speedFactorLeft,
  (float)(0) maxSpeedRight,
  (float)(1) speedFactorRight,
  (Angle)(0) predFallDownAngleFront,
  (Angle)(0) predFallDownAngleBack,
  (Angle)(0) predFallDownAngleLeft,
  (Angle)(0) predFallDownAngleRight,
  (Vector2f)(Vector2f::Ones()) fallDownSpeedReductionFactor
);

STREAMABLE(HeatStatus,,
  (unsigned char) kneePitchLeftStat,
  (unsigned char) kneePitchRightStat,
  (unsigned char) anklePitchLeftStat,
  (unsigned char) anklePitchRightStat
);

STREAMABLE(MotionState,
  void draw() const;
  ENUM(MotionStateError,
    engineKickFailure,
    customKickFailure,
    walkingForwardFailure,
    walkingBackwardFailure,
    walkingLeftFailure,
    walkingRightFailure,
    legLeftStiffness,
    legRightStiffness,
    motionFrameRateAbnormal,
    cognitionFrameRateAbnormal,
    kneePitchLeftTooHot,
    kneePitchRightTooHot,
    anklePitchLeftTooHot,
    anklePitchRightTooHot
  ),

  (FsrStatus) fsrStatus,
  (IMUStatus) imuStatus,
  (JointStatus) jointStatus,
  (FrameRateStatus) frameRateStatus,
  (WalkingStatus) walkingStatus,
  (HeatStatus) heatStatus,
  (std::vector<MotionStateError>) motionProblems
);
