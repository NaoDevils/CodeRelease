/**
 * @file Modules/MotionControl/KeyFrameEngine.h
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/JointError.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Tools/ProcessFramework/CycleLocal.h"
#include "Tools/MessageQueue/InMessage.h"
#include <array>
#include <queue>

// tools
#include "Tools/RingBufferWithSum.h"


STREAMABLE(KeyFrameMotion,
  STREAMABLE(KeyFrame,
    ENUM(KeyFrameInterpolationType,
      linear,
      sine
    )

    void mirror();
    ,

    (std::array<Angle, 2>) headAngles,
    (std::array<Angle, 12>) armsAngles,
    (std::array<Angle, 12>) legsAngles,
    (std::array<int, Joints::numOfJoints>) stiffnesses,
    (unsigned int)(100) duration,
    (KeyFrameInterpolationType)(linear) intType,
    (Angle)(0_deg) angleAtKeyFrameTarget,
    (Angle)(30_deg) angleAtKeyFrameErrorFront,
    (Angle)(30_deg) angleAtKeyFrameErrorBack,
    (bool)(false) useAngleAtKeyFrameTarget,
    (bool)(false) activateFallDownProtection,
    (bool)(false) stabilize,
    (bool)(false) waitForStable,
    (bool)(false) leavingPossible,
    ((SpecialActionRequest) SpecialActionID)(none) fallDownProtection,
    (unsigned)(2000) holdFallDownProtection
  ),
  ((SpecialActionRequest) SpecialActionID)(playDead) keyFrameID,
  (bool) (false) initialKeyFrame,
  ((SpecialActionRequest) SpecialActionID)(none) nextKeyFrameID,
  (float) (0.f) YstabilizationP,
  (float) (0.f) YstabilizationI,
  (float) (0.f) YstabilizationD,
  (float) (0.f) XstabilizationP,
  (float) (0.f) XstabilizationI,
  (float) (0.f) XstabilizationD,
  (std::vector<KeyFrame>) keyFrames
);

MODULE(KeyFrameEngine,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  REQUIRES(JointError),
  REQUIRES(JointSensorData),
  REQUIRES(MotionSelection),
  REQUIRES(MotionRequest),
  REQUIRES(StiffnessSettings),
  REQUIRES(JoinedIMUData),
  REQUIRES(FsrSensorData),
  REQUIRES(RobotInfo),
  REQUIRES(FallDownState),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(MassCalibration),
  REQUIRES(WalkingEngineParams),
  PROVIDES(SpecialActionsOutput),
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (int) maxWaitForStableTime, /* After this time, the KeyFrameMotion does not waitForStable gyro anymore. */
    (int) maxWaitForArmsTime, /* After this time, the KeyFrameMotion does not waitForArms anymore. */
    (Angle) maxAvgGyroForStable, /* If avg gyro exceeds this, waitForStable KeyFrames are not finished. */
    (bool) (false) useStuckArmsDetection,
    (Angle) (15_deg) maxAngleXForStable, 
    (Angle) (30_deg) maxArmError, 
    (Angle) (90_deg) bodyAngleOnGround, 
    (Angle) (5_deg) angleDiffOnGround, 
    (std::array<int, Joints::numOfJoints>) defaultStiffnesses, /* Stiffness values are initialized with this. */
    (Angle)(10_deg) maxAngleDifference, /**< Maximum angle difference of body y to the motion's 'angleAtKeyFrameTarget'. Motion can be stopped if exceeded. */
    (std::array<Angle, Joints::numOfJoints>) degPerFrame, /**< Maximum allowed degree change per frame. */
    (bool) (false) debugModeForWalkingCalibration, /* Whether to use the fall down protection or not. */
    (float) (0.f) ankleInfluence, /* How much the PID values affect the ankle. */
    (float) (0.f) kneeInfluence, /* How much the PID values affect the knee. */
    (float) (0.f) hipInfluence, /* How much the PID values affect the hip. */
    (float) (0.f) ankleRollInfluence, /* How much the PID values affect the ankle. */
    (float) (0.f) hipRollInfluence, /* How much the PID values affect the hip. */
    (int) (750) saveFallFrontTime,
    (int) (750) saveFallBackTime,
    (int) (750) saveFallTime,
    (int) (1000) lyingTime,
    (bool)(false) textOutput /* True to print if Joint moves faster*/
  )
);

class KeyFrameEngine : public KeyFrameEngineBase
{
private:
  void update(SpecialActionsOutput& specialActionsOutput);


  void init(SpecialActionsOutput& specialActionsOutput); // reset data when not initialized
  void initEngineData(SpecialActionsOutput& specialActionsOutput); // reset engine data at start/end of activation
  bool interpolate(SpecialActionsOutput& specialActionsOutput); // interpolate last key frame to current key frame with intType
  void stabilize(SpecialActionsOutput& specialActionsOutput); // if keyFrame wants stabilization, use leg pitches to stablize
  bool isStable(); // checks if gyro values are stable
  bool armsStuck(); // check if arms get stuck
  KeyFrameMotion::KeyFrame setKeyFrameAngles(const KeyFrameMotion::KeyFrame& keyFrame); // sets key frame angles for \c keyFrame.

  // select specific KeyFrameMotion, fall back to default (stand) if not found
  void selectActiveMotion(SpecialActionRequest::SpecialActionID id, bool mirror, SpecialActionsOutput& specialActionsOutput);

  std::queue<bool> specialActionMirrorQueue;
  std::queue<unsigned> specialActionTimerQueue;
  std::queue<SpecialActionRequest::SpecialActionID> specialActionIDQueue;
  bool areMotionsPrioritized();
  void priorizeMotion(SpecialActionRequest::SpecialActionID id, bool mirror, unsigned timer = 12);
  void clearPrioritizedMotions();
  void selectPrioritizedMotion(SpecialActionsOutput& specialActionsOutput);

  /* called at end of interpolation of key frame.
   * checks if upper body y angle or any arm/leg angle is too different from desired angle. 
  **/
  bool verifyRobotPosition(SpecialActionsOutput& specialActionsOutput);
  bool isMotionStable(SpecialActionsOutput& specialActionsOutput);
  void handleTransitions(SpecialActionsOutput& specialActionsOutput);
  bool resetFallDownProtectionNeeded(SpecialActionsOutput& specialActionsOutput);
  void handleInitialFallDownProtection();
  void chooseFallDownProtection(SpecialActionRequest::SpecialActionID id = SpecialActionRequest::none, unsigned timer = 12, bool advancedProtection = true);
  bool initialFallDownProtectionActive = false;

  bool handleMessage2(InMessage& message);

  static KeyFrameMotion::KeyFrame getKeyFrameFromJointRequest(const JointRequest& jointRequest);

  // members
  bool debugMode = false; // if active, key frames will be played one by one via debug request
  bool initialized = false; // only initialize once
  bool engineDataReset = false; /* Whether the engine data including phase, angles and sensor buffers was reset. */
  //bool abortMotion = false; /* Whether the current KeyFrameMotion should be aborted. Not used at the moment. */
  std::vector<KeyFrameMotion> keyFrameMotions; // overwritten by loadKeyFrameMotions()

  int currentKeyFrameMotionIndex = 0; /* The index of the active KeyFrameMotion in the vector of all KeyFrameMotions. */
  int currentKeyFrameIndex = -1; /* The index of the active KeyFrame in the vector of all KeyFrames in the current KeyFrameMotion. */
  bool currentKeyFrameMirror = false; /* If the current motion is mirrored. */
  KeyFrameMotion::KeyFrame originFrame; /* The angles with which the current KeyFrameMotion started. */
  KeyFrameMotion::KeyFrame lastKeyFrame; /* The angles at the end of the last KeyFrame in the current KeyFrameMotion. */
  float phase = 0.f; /**< The phase within KeyFrame interpolation. [0..1] */
  bool keyFrameWasFinished = false; /**< Set back frame finishing at start of new one. */
  unsigned keyFrameFinishedTimestamp = 0; /**< When did the last KeyFrame finish? Used for waitForStable. */
  unsigned keyFrameMotionFinishedTimestamp = 0;
  // remember the specific KeyFrameMotion and KeyFrame for the above timestamp
  int lastFinishedKeyFrameMotionIndex = 0;
  int lastFinishedKeyFrameIndex = -1;
  bool prevArmsStuck = true;
  bool prevFlying = false;

  SpecialActionRequest::SpecialActionID initialKeyFrameMotionID = SpecialActionRequest::none;

  // for sensor control
  RingBufferWithSum<Angle, 5> angleYBuffer;
  RingBufferWithSum<Angle, 5> gyroXBuffer;
  RingBufferWithSum<Angle, 5> gyroYBuffer;
  RingBufferWithSum<Angle, 5> hipYawBuffer;
  RingBufferWithSum<Angle, 5> lAnkleRollBuffer;
  RingBufferWithSum<Angle, 5> rAnkleRollBuffer;
  JointAngles jointAngleBuffer[5];
  int currentJointAngleID = 0;

  static CycleLocal<KeyFrameEngine*> theInstance;

public:
  /*
  * Default constructor.
  */
  KeyFrameEngine();
  ~KeyFrameEngine() { theInstance.reset(); }

  static std::vector<KeyFrameMotion> loadKeyFrameMotions(); // load all .kfm files from Config/KeyFrameEngine folder
  static bool handleMessage(InMessage& message);
};
