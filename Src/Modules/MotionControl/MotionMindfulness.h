/**
* @file Modules/MotionControl/MotionMindfulness.h
* This file declares a module that is responsible for self diagnosis of the robots body functions.
* @author <A href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
* @author <A href="mailto:dominik.braemer@tu-dortmund.de">Dominik Brämer</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include <cfloat>
#include <tuple>
#include <vector>

// input
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/Sensing/BrokenJointState.h"
#include "Modules/MotionControl/DortmundWalkingEngine/CSConverter2019.h"

// output
#include "Representations/MotionControl/MotionState.h"

// tools
#include "Tools/RingBufferWithSum.h"

MODULE(MotionMindfulness,
  // low level input
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData),
  REQUIRES(JoinedIMUData),
  REQUIRES(FsrSensorData),

  // high level input
  REQUIRES(GameInfo),
  REQUIRES(FallDownState),
  REQUIRES(MotionRequest),
  REQUIRES(MotionInfo), // for custom step kick
  REQUIRES(RobotHealth),
  REQUIRES(SpeedInfo),
  REQUIRES(SpeedRequest),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  REQUIRES(KinematicRequest),
  REQUIRES(BrokenJointState),
  PROVIDES(MotionState),
  LOADS_PARAMETERS(,
    (bool)(true) activate,
    (bool)(true) enableName,

    (int)(3000) soundOutputTime, // Sound wait time for a detected defect. 

    // stiffness problem detection parameters -------------------------------------------
    (int)(50) maxFramesWithoutJointActivation,
    (Angle)(2_deg) minJointDiff,

    // broken joints detection parameters -------------------------------------------------
    (int)(60) maxArmErrors,
    (int)(50) maxLegErrors,

    // fsr status detection parameters --------------------------------------------------
    (int)(60) maxFsrErrors,

    // imu status detection parameters --------------------------------------------------
    (int)(30) maxImuErrors,

    // walking problem detection parameters ---------------------------------------------
    (Angle)(5_deg) maxAngleWalkingSidewardsX,
    (Angle)(5_deg) maxAngleWalkingSidewardsY,
    (int)(50) maxSpeedForwardBackward, // used to detect sidewards walking
    (Angle)(5_deg) maxAngleWalkingForwardsX,
    (Angle)(5_deg) maxAngleWalkingForwardsY,
    (Angle)(5_deg) maxAngleWalkingBackwardsX,
    (Angle)(5_deg) maxAngleWalkingBackwardsY,
    (int)(50) maxSpeedSideward, // used to detect backwards walking

    // max number of unstable frames
    // maxAllowedUnstableFrames - minUnstableFrames are the frames 
    // that must be stable before failure is reset
    (int)(250) maxAllowedUnstableFrames,

    // frames that must be unstable before detection is triggered
    (int)(30) minUnstableFrames,

    // kick problem detection parameters ------------------------------------------------
    (unsigned)(5000) timeAfterEngineKick, // time in milliseconds
    (unsigned)(5000) timeAfterCustomKick, // time in milliseconds

    // fall angle detection parameters --------------------------------------------------
    (float)(2.5) minWeight,

    // walkspeed factor prediction parameters -------------------------------------------
    (float)(0.99f) upscaleThreshold,
    (float)(0.75f) downscaleThreshold,
    (float) (10.0f) initialDecreaseIncreaseFactor,
    (float) (1.0f) minDecreaseIncreaseFactor,
    (float) (0.1f) annealingFactor,
    (float) (25.0f) minSpeedForDetection,
    (unsigned) (3000) resetWalkingErrorThreshold,
    (float)(0.2f) kinematicStumblingWeight,
    (float)(0.6f) angleStumblingWeight,
    (float)(0.2f) fsrStumblingWeight,
    (float)(0.75f) stumblingThreshold,

    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource
   )
);

class MotionMindfulness : public MotionMindfulnessBase
{
public:
  MotionMindfulness();
  ~MotionMindfulness();

private:
  void update(MotionState& theMotionState);

  bool hasProblem(MotionState& motionState, MotionState::MotionStateError motionStateError);

  void checkForBrokenJoints(MotionState& motionState);
  void checkForFsrSanity(MotionState& motionState);
  void checkForImuSanity(MotionState& motionState);
  void checkFramerate(MotionState& motionState);
  void checkForWalkProblems(MotionState& motionState);
  void checkForKickProblems(MotionState& motionState);
  void checkForHeatProblems(MotionState& motionState);

  void stepTimeDetection(MotionState& motionState);
  void fallAngleDetection(MotionState& motionState);

  void walkSpeedProtocol(MotionState& motionState);
  void walkSpeedFactorPrediction(MotionState& motionState);

  void kinematicStumbleDetection(MotionState& motionState);
  void angleVarianceStumbleDetection(MotionState& motionState);
  void fsrVarianceStumbleDetection(MotionState& motionState);
  void stumbleDetectionAccumulator(MotionState& motionState);

  void updateWalkErrors(MotionState& motionState, const bool increase);

  MotionState localMotionState;


  std::string robotName;
  // member variables
  // ----------------
  // CSConverter2019 parameters
  CSConverter2019::Parameters csConverterParams;

  // broken joint detection
  std::vector<int> brokenJointErrors;
  std::vector<int> stuckJointErrors;

  // walk problem detection
  int errorsWalking[4] = {0};

  // kick problem detection
  unsigned lastEngineKickTime;
  unsigned lastCustomKickTime;

  // fsr sanity check
  float fsrLeft[FsrSensorData::numOfFsrSensorPositions];
  float fsrRight[FsrSensorData::numOfFsrSensorPositions];
  int errorsFsrLeft[FsrSensorData::numOfFsrSensorPositions] = {0};
  int errorsFsrRight[FsrSensorData::numOfFsrSensorPositions] = {0};

  // imu sanity check
  float acc[3];
  float gyro[3];
  float angle[2];
  int errorsAcc[3] = {0};
  int errorsGyro[3] = {0};
  int errorsAngle[2] = {0};

  // frame rate check
  int motionFrameError = 0;
  int cognitionFrameError = 0;
  int frequency = 0;

  // heat check
  unsigned char lastStateKneePitchRight = 0;
  unsigned char lastStateKneePitchLeft = 0;
  unsigned char lastStateAnklePitchRight = 0;
  unsigned char lastStateAnklePitchLeft = 0;

  // step Time detection
  RingBufferWithSum<unsigned, 10> stepTime;
  int beginStepLeft = 0;
  int beginStepRight = 0;
  int endStepLeft = 0;
  int endStepRight = 0;
  bool lastFsrLeft = false;
  bool lastFsrRight = false;
  float minFsrLeft = 5.0f;
  float minFsrRight = 5.0f;

  // fall Angle detection
  RingBufferWithSum<Angle, 833> bufferAngleX; //Buffer last 10 seconds
  RingBufferWithSum<Angle, 833> bufferAngleY;
  Angle maxAngleX;
  Angle minAngleX;
  Angle maxAngleY;
  Angle minAngleY;

  // speed detection
  unsigned countSpeedForward = 0;
  unsigned countSpeedBackward = 0;
  unsigned countSpeedLeft = 0;
  unsigned countSpeedRight = 0;

  RingBufferWithSum<float, 833> bufferSpeedForward; //Buffer last 10 seconds
  RingBufferWithSum<float, 833> bufferSpeedBackward;
  RingBufferWithSum<float, 833> bufferRequestSpeedForward; //Buffer last 10 seconds
  RingBufferWithSum<float, 833> bufferRequestSpeedBackward;

  RingBufferWithSum<float, 250> bufferSpeedLeft; //Buffer last 3 seconds
  RingBufferWithSum<float, 250> bufferSpeedRight;
  RingBufferWithSum<float, 250> bufferRequestSpeedLeft; //Buffer last 3 seconds
  RingBufferWithSum<float, 250> bufferRequestSpeedRight;

  bool bufferForwardFull = false;
  bool bufferBackwardFull = false;
  bool bufferLeftFull = false;
  bool bufferRightFull = false;

  bool bufferForwardFullAndStable = false;
  bool bufferBackwardFullAndStable = false;
  bool bufferLeftFullAndStable = false;
  bool bufferRightFullAndStable = false;

  bool forwardUpdated = false;
  bool backwardUpdated = false;
  bool leftUpdated = false;
  bool rightUpdated = false;

  float lastMaxForward = 0.0f;
  float lastMaxBackward = 0.0f;
  float lastMaxRequestForward = 0.0f;
  float lastMaxRequestBackward = 0.0f;

  float lastMaxLeft = 0.0f;
  float lastMaxRight = 0.0f;
  float lastMaxRequestLeft = 0.0f;
  float lastMaxRequestRight = 0.0f;

  float maxSpeedForward = 0.0f;
  float maxSpeedBackward = 0.0f;
  float maxSpeedRequestForward = 0.0f;
  float maxSpeedRequestBackward = 0.0f;

  float maxSpeedLeft = 0.0f;
  float maxSpeedRight = 0.0f;
  float maxSpeedRequestLeft = 0.0f;
  float maxSpeedRequestRight = 0.0f;

  float decraseIncreaseFactorForward = 10.0f;
  float decraseIncreaseFactorBackward = 10.0f;
  float decraseIncreaseFactorLeft = 10.0f;
  float decraseIncreaseFactorRight = 10.0f;

  unsigned startTimestamp = 0;

  bool usableKinematicOffsets = false;
  float maxPitchOffset = 0.f;
  float maxRollOffset = 0.f;
  RingBuffer<float, 417> bufferKinematicPitchOffsets;
  RingBuffer<float, 417> bufferKinematicRollOffsets;

  bool usableAngleValues = false;
  float minVarianceX = FLT_MAX;
  float maxVarianceX = -FLT_MAX;
  float minVarianceY = FLT_MAX;
  float maxVarianceY = -FLT_MAX;
  RingBuffer<float, 167> angleValuesX;
  RingBuffer<float, 167> angleValuesY;
  RingBuffer<float, 250> bufferAngleValuesX;
  RingBuffer<float, 250> bufferAngleValuesY;

  bool usableFsrValues = false;
  std::array<float, FsrSensorData::numOfFsrSensorPositions> minVarianceLeft = {FLT_MAX};
  std::array<float, FsrSensorData::numOfFsrSensorPositions> maxVarianceLeft = {-FLT_MAX};
  std::array<float, FsrSensorData::numOfFsrSensorPositions> minVarianceRight = {FLT_MAX};
  std::array<float, FsrSensorData::numOfFsrSensorPositions> maxVarianceRight = {-FLT_MAX};
  RingBuffer<std::array<float, FsrSensorData::numOfFsrSensorPositions>, 167> fsrValuesLeft;
  RingBuffer<std::array<float, FsrSensorData::numOfFsrSensorPositions>, 167> fsrValuesRight;
  RingBuffer<std::array<float, FsrSensorData::numOfFsrSensorPositions>, 250> bufferFsrValuesLeft;
  RingBuffer<std::array<float, FsrSensorData::numOfFsrSensorPositions>, 250> bufferFsrValuesRight;

  bool walkingForward = false;
  bool walkingBackward = false;
  bool walkingLeft = false;
  bool walkingRight = false;

  unsigned lastUpdatedWalkingError[4] = {0};
};
