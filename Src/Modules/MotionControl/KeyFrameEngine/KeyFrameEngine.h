/**
 * @file Modules/MotionControl/KeyFrameEngine.h
 * This file declares a module that creates the motions of the stand.
 * @author <a href="aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Modeling/IMUModel.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/RobotModel.h"

// tools
#include "Tools/RingBufferWithSum.h"


STREAMABLE(KeyFrameMotion,
{
  STREAMABLE(KeyFrame,
  {
    ENUM(KeyFrameInterpolationType,
    { ,
      linear,
      sine,
    }),

    (Angle[2]) headAngles,
    (Angle[12]) armsAngles,
    (Angle[12]) legsAngles,
    (std::array<int, Joints::numOfJoints>) stiffnesses,
    (unsigned int)(100) duration,
    (KeyFrameInterpolationType)(linear) intType,
    (Angle)(0_deg) angleAtKeyFrameTarget,
    (bool)(false) useAngleAtKeyFrameTarget,
    (bool)(false) stabilize,
    (bool)(false) waitForStable,
    (bool)(false) leavingPossible,
  }),
  ((SpecialActionRequest) SpecialActionID)(playDead) keyFrameID,
  (float) (0.f) stabilizationP,
  (float) (0.f) stabilizationI,
  (float) (0.f) stabilizationD,
  (std::vector<KeyFrame>) keyFrames,
});

MODULE(KeyFrameEngine,
{ ,
  REQUIRES(FrameInfo),
  REQUIRES(JointCalibration),
  REQUIRES(JointAngles),
  REQUIRES(MotionSelection),
  REQUIRES(StiffnessSettings),
  REQUIRES(IMUModel),
  REQUIRES(InertialSensorData),
  REQUIRES(FsrSensorData),
  REQUIRES(RobotInfo),
  REQUIRES(MotionRequest),
  REQUIRES(RobotModel),
  PROVIDES(SpecialActionsOutput),
  LOADS_PARAMETERS(
  { ,
    (bool)(true) useHipYawCorrection, /* If true, add hipYaw error to hipPitch to keep robot upright. */
    (bool)(true) useIMUModel, /* If true, IMUModel is used for sensor control. */
    (int) maxWaitForStableTime, /* After this time, the KeyFrameMotion does not waitForStable gyro anymore. */
    (Angle) maxAvgGyroForStable, /* If avg gyro exceeds this, waitForStable KeyFrames are not finished. */
    (std::array<int, Joints::numOfJoints>) defaultStiffnesses, /* Stiffness values are initialized with this. */
    (Angle)(10_deg) maxAngleDifference, /**< Maximum angle difference of body y to the motion's 'angleAtKeyFrameTarget'. Motion can be stopped if exceeded. */
    
    (float) (0.f) ankleInfluence, /* How much the PID values affect the ankle. */
    (float) (0.f) kneeInfluence, /* How much the PID values affect the knee. */
    (float) (0.f) hipInfluence, /* How much the PID values affect the hip. */
    (float) (0.f) hipYawCorrectionP, /* If useHipYawCorrection is true, the hipYaw error affects the hipPitch this much. */
    (Angle) (-67_deg) minAnklePitchAngle, 
  }),
});

class KeyFrameEngine : public KeyFrameEngineBase
{
private:
  void update(SpecialActionsOutput& specialActionsOutput);

  
  void init(SpecialActionsOutput& specialActionsOutput); // reset data when not initialized 
  void initEngineData(SpecialActionsOutput& specialActionsOutput); // reset engine data at start/end of activation
  bool interpolate(SpecialActionsOutput& specialActionsOutput); // interpolate last key frame to current key frame with intType
  void stabilize(SpecialActionsOutput& specialActionsOutput); // if keyFrame wants stabilization, use leg pitches to stablize
  bool isStable(); // checks if gyro values are stable
  void loadKeyFrameMotions(); // load all .kfm files from Config/KeyFrameEngine folder
  KeyFrameMotion::KeyFrame setKeyFrameAngles(const KeyFrameMotion::KeyFrame& keyFrame); // sets key frame angles for \c keyFrame.

  // select specific KeyFrameMotion, fall back to default (stand) if not found
  void selectActiveMotion(SpecialActionRequest::SpecialActionID id, SpecialActionsOutput& specialActionsOutput); 
  
  /* called at end of interpolation of key frame.
   * checks if upper body y angle or any arm/leg angle is too different from desired angle. 
  **/
  bool verifyRobotPosition(SpecialActionsOutput& specialActionsOutput); 

  
  // members
  bool initialized = false; // only initialize once
  bool engineDataReset = false; /* Whether the engine data including phase, angles and sensor buffers was reset. */
  bool abortMotion = false; /* Whether the current KeyFrameMotion should be aborted. Not used at the moment. */
  std::vector<KeyFrameMotion> keyFrameMotions; // overwritten by loadKeyFrameMotions()

  int currentKeyFrameMotionIndex = 0; /* The index of the active KeyFrameMotion in the vector of all KeyFrameMotions. */
  int currentKeyFrameIndex = -1; /* The index of the active KeyFrame in the vector of all KeyFrames in the current KeyFrameMotion. */
  KeyFrameMotion::KeyFrame originFrame; /* The angles with which the current KeyFrameMotion started. */
  KeyFrameMotion::KeyFrame lastKeyFrame; /* The angles at the end of the last KeyFrame in the current KeyFrameMotion. */
  float phase = 0.f; /**< The phase within KeyFrame interpolation. [0..1] */
  unsigned keyFrameFinishedTimestamp = 0; /**< When did the last KeyFrame finish? Used for waitForStable. */
  // remember the specific KeyFrameMotion and KeyFrame for the above timestamp
  int lastFinishedKeyFrameMotionIndex = 0;
  int lastFinishedKeyFrameIndex = -1;

  // for sensor control
  RingBufferWithSum<Angle, 5> angleYBuffer;
  RingBufferWithSum<Angle, 5> gyroYBuffer;
  RingBufferWithSum<Angle, 5> hipYawBuffer;

public:
  /*
  * Default constructor.
  */
  KeyFrameEngine();

};
