/**
 * @file Modules/MotionControl/KeyFrameEngine.h
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SensorControlParams.h"
#include "Representations/MotionControl/JointError.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Tools/ProcessFramework/CycleLocal.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Joints.h"
#include <array>
#include <vector>
#include <queue>
#include <tuple>
#include <map>

// tools
#include "Tools/RingBufferWithSum.h"


STREAMABLE(KeyFrameMotion,
  

  STREAMABLE(KeyFrame,
    ENUM(KeyFrameInterpolationType,
      linear,
      sine
    )
    ENUM(Conditions,
        execute,
        lyingOnFront,
        lyingOnBack,
        lyingOnSide,
        notOnGround,
        mightUpright,
        notLying,
        useStandUpStatistic,
        none
    )
    using ConditionVector = std::vector<Conditions>;
    void mirror();
    ,

    (std::array<Angle, 2>) headAngles,
    (std::array<Angle, 12>) armsAngles,
    (std::array<Angle, 12>) legsAngles,
    (std::array<int, Joints::numOfJoints>) stiffnesses,
    (unsigned int)(100) duration,
    (KeyFrameInterpolationType)(linear) intType,
    (Angle)(0_deg) angleAtKeyFrameTarget,
    (std::array<Angle, 2>) angleAtKeyFrameError,
    (bool)(false) stabilize,
    (bool)(false) waitForStable,
    (bool)(false) armAngleReached,
    (bool)(false) leavingPossible,
    (std::array<bool,2>) useArmProblemDetection,
    (unsigned int)(0) holdFrame, 
    ((KeyFrame) ConditionVector) nextKeyFrameConditions,
    ((SpecialActionRequest) MotionIDVector) nextKeyFrameIDs,
    ((SpecialActionRequest) SpecialActionID)(none) fallDownProtection,
    (unsigned int)(2000) holdFallDownProtection
  ),
  ((SpecialActionRequest) SpecialActionID)(playDead) keyFrameID,
  (bool) (false) initialKeyFrame,
  ((KeyFrame) ConditionVector) nextMotionConditions,
  ((SpecialActionRequest) MotionIDVector) nextKeyFrameMotionIDs,
  (float) (0.f) YstabilizationP,
  (float) (0.f) YstabilizationI,
  (float) (0.f) YstabilizationD,
  (float) (0.f) XstabilizationP,
  (float) (0.f) XstabilizationI,
  (float) (0.f) XstabilizationD,
  (float) (2.f) errorMultiplier,
  (float) (.05f) reductionRate,
  (Angle) (10_deg) maxCompensation,
  (Angle) (4_deg) compensationThreshold,
  (std::vector<float>) hipYawCorrectionFactors,
  (std::vector<float>) hipCorrectionFactors,
  (std::vector<float>) kneeCorrectionFactors,
  (std::vector<float>) ankleCorrectionFactors,
  (Angle) (10_deg) localArmAngleReachedThreshold,
  (std::vector<KeyFrame>) keyFrames
);

MODULE(KeyFrameEngine,
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(JointError),
  REQUIRES(JointSensorData),
  REQUIRES(MotionSelection),
  REQUIRES(MotionRequest),
  REQUIRES(JoinedIMUData),
  REQUIRES(RobotInfo),
  REQUIRES(FallDownState),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(MassCalibration),
  REQUIRES(WalkingEngineParams),
  REQUIRES(SensorControlParams),
  PROVIDES(SpecialActionsOutput),
  LOADS_PARAMETERS(,
    ((JoinedIMUData) InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (int) maxWaitForStableTime, /* After this time, the KeyFrameMotion does not waitForStable gyro anymore. */
    (int) maxWaitForArmsTime, /* After this time, the KeyFrameMotion does not waitForArms anymore. */
    (int) (500) maxWaitForArmAngleTime,
    (Angle) maxAvgGyroForStable, /* If avg gyro exceeds this, waitForStable KeyFrames are not finished. */
    (Angle) (15_deg) maxAngleXForStable, 
    (Angle) (30_deg) maxShoulderRollError,
    (Angle) (15_deg) maxShoulderPitchError,
    (Angle) (15_deg) maxElbowError,
    (float) (0.75f) standUpPossibleThreshold,
    (int) (3) minStandUpTries,
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
    (Angle) (15_deg) sitAbortThreshold,
    (int) (1000) mightUprightTime,
    ((SpecialActionRequest) MotionIDVector) ignoreInitialKeyFrameIfChained,
    (bool) (true) ignoreStandUpNotWorkingInSimulator,
    (bool)(false) textOutput /* True to print if Joint moves faster*/
  )
);

class KeyFrameEngine : public KeyFrameEngineBase
{
private:
  enum ArmsStuckState
  {
    fine,
    stuck,
    lyingOnArm
  };
  void update(SpecialActionsOutput& specialActionsOutput);

  void plotAngleAtKeyframeTarget(SpecialActionsOutput& specialActionsOutput);

  void init(SpecialActionsOutput& specialActionsOutput); // reset data when not initialized
  void initEngineData(SpecialActionsOutput& specialActionsOutput); // reset engine data at start/end of activation
  bool interpolate(SpecialActionsOutput& specialActionsOutput); // interpolate last key frame to current key frame with intType
  void stabilize(SpecialActionsOutput& specialActionsOutput); // if keyFrame wants stabilization, use leg pitches to stablize
  bool isStable(); // checks if gyro values are stable
  bool checkCondition(KeyFrameMotion::KeyFrame::Conditions condition); // is condition true or false
  SpecialActionRequest::SpecialActionID selectNextMotionID(std::vector<SpecialActionRequest::SpecialActionID> nextMotionIDs, std::vector<KeyFrameMotion::KeyFrame::Conditions> conditions); // select next keyFrame / keyFrameMotionID
  ArmsStuckState armsStuck(); // check if arms get stuck
  bool armAngleReached(SpecialActionsOutput& specialActionsOutput);
  KeyFrameMotion::KeyFrame setKeyFrameAngles(const KeyFrameMotion::KeyFrame& keyFrame); // sets key frame angles for \c keyFrame.

  bool standUpMotionActive();
  bool inStandUpMotion();
  void calculateStandUpStatistic(SpecialActionsOutput& specialActionsOutput);
  std::vector<std::vector<int>> countStandUpStarts;
  std::vector<std::vector<int>> countStandUpAborts;
  std::vector<std::vector<float>> standUpStatistic;
  int lastStandUpMotionIndex = 0;
  bool standUpStarted = false;
  bool standUpNotWorking = false;
  bool standUpSideLegit = false;
  bool ignoreInitialKeyFrame = false;
  int armsStuckCount = 0;

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
  void handleInitialFallDownProtection(SpecialActionsOutput& specialActionsOutput);
  void chooseFallDownProtection(SpecialActionsOutput& specialActionsOutput, SpecialActionRequest::SpecialActionID id = SpecialActionRequest::none, unsigned timer = 12);
  bool initialFallDownProtectionActive = false;

  static bool sortJoints(std::tuple<Angle, Joints::Joint>& a, std::tuple<Angle, Joints::Joint>& b) { return std::abs(std::get<0>(a)) > std::abs(std::get<0>(b)); }
  void gatherRequestDiff(SpecialActionsOutput& specialActionsOutput, bool afterInterpolation);
  void compensateJointError(SpecialActionsOutput& specialActionsOutput);
  bool compensateHipError(SpecialActionsOutput& specialActionsOutput);
  void compensateLegError(SpecialActionsOutput& specialActionsOutput, bool leftLeg);

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
  unsigned int globalFrontBackIdx = 0;
  bool prevArmsStuck = true;
  bool prevFlying = false;
  bool standTransitionActive = false;
  bool stabilizeWarningActive = false;

  unsigned lastPhaseUpdateTimestamp = 0;
  unsigned currentCycleTime = 0;

  SpecialActionRequest::SpecialActionID initialKeyFrameMotionID = SpecialActionRequest::none;
  SpecialActionRequest::SpecialActionID lastStandUpStartMotion = SpecialActionRequest::none;

  float lCompensationReductionFactor = 1.f;
  float rCompensationReductionFactor = 1.f;
  std::map<Joints::Joint, Angle> jointRequestDiff{{Joints::lHipYawPitch, 0_deg},
      {Joints::lHipPitch, 0_deg},
      {Joints::lKneePitch, 0_deg},
      {Joints::lAnklePitch, 0_deg},
      {Joints::rHipYawPitch, 0_deg},
      {Joints::rHipPitch, 0_deg},
      {Joints::rKneePitch, 0_deg},
      {Joints::rAnklePitch, 0_deg}};
  RingBufferWithSum<float, 3> lJointErrorBuffer;
  RingBufferWithSum<float, 3> rJointErrorBuffer;
  RingBufferWithSum<float, 3> jointErrorBuffer;

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
