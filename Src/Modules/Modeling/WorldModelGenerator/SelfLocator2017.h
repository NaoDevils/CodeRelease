/**
* @file Modules/SelfLocator/SelfLocator2017.h
*
* State in January 2017: removed SLAM stuff (was not used ever).
* Original implementation can be found in NDD CodeRelease 2013.
* 
* @author <a href="mailto:stefan.tasse@tu-dortmund.de">Stefan Tasse</a>
* @author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
*
*/

#pragma once

//#include <algorithm>
#include <vector>
#include <memory>

// ------------- NAO-Framework includes --------------
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/BehaviorControl/VisualRefereeBehaviorSymbols.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPoseHypotheses.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/LineMatchingResult.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"

#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/GaussianDistribution3D.h"

#include "Tools/Modeling/PoseGenerator.h"

// ------------- World Model Generator includes --------------
#include "models/PoseHypothesis2017.h"
#include "SelfLocator2017Parameters.h"

constexpr unsigned IMU_BUFFER_LENGTH = static_cast<unsigned>(0.5 * 83);

MODULE(SelfLocator2017,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FrameInfo),
  REQUIRES(TeammateData),
  REQUIRES(FieldDimensions),
  REQUIRES(JoinedIMUData),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OdometryData),
  REQUIRES(LineMatchingResult),
  REQUIRES(CLIPCenterCirclePercept),
  REQUIRES(CLIPGoalPercept),
  REQUIRES(PenaltyCrossPercept),
  REQUIRES(BallModel),
  REQUIRES(FallDownState),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(RemoteBallModel),
  USES(VisualRefereeBehaviorSymbols),
  USES(PositioningSymbols),
  USES(LocalRobotMap),
  USES(RemoteRobotMap),
  USES(BehaviorData), // For manual positions, role is required (right now only goalie vs field player)
  PROVIDES(RobotPose),
  PROVIDES_WITHOUT_MODIFY(RobotPoseHypotheses), //all
  PROVIDES_WITHOUT_MODIFY(RobotPoseHypothesesCompressed), //all compressed (for logging)
  PROVIDES_WITHOUT_MODIFY(RobotPoseHypothesis), //the best
  PROVIDES(SideConfidence),
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    ((JoinedIMUData)InertialDataSource)(JoinedIMUData::inertialSensorData) anglesource,
    (Angle)(0.01_deg) gyroMaxVariance,
    (PositionsByRules) positionsByRules,
    (SelfLocator2017Parameters) parameters
  )
);


class SelfLocator2017 : public SelfLocator2017Base
{
  using PoseHypotheses = std::vector<std::unique_ptr<PoseHypothesis2017>>;

  struct HypothesisBase
  {
    Pose2f pose;
    float positionConfidence;
    SideConfidence::ConfidenceState confidenceState;

    HypothesisBase(const Pose2f& _pose, float _positionConfidence, SideConfidence::ConfidenceState _confidenceState)
        : pose(_pose), positionConfidence(_positionConfidence), confidenceState(_confidenceState)
    {
    }
  };

  ENUM(LocalizationState,

      positionTracking,
      fallenDown,
      positionLost,
      penalized)

public:
  /*------------------------------ public methods -----------------------------------*/

  /** 
  * Constructor.
  */
  SelfLocator2017();

  ~SelfLocator2017();

  /**
  * The function executes the module.
  */
  void update(RobotPose& robotPose);
  void update(RobotPoseHypothesis& robotPoseHypothesis);
  void update(RobotPoseHypotheses& robotPoseHypotheses);
  void update(RobotPoseHypothesesCompressed& robotPoseHypotheses);
  void update(SideConfidence& sideConfidence);

private:
  /* ----------------------------------- private variables here ----------------------------------*/

  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersX;
  std::array<RingBufferWithSum<Angle, IMU_BUFFER_LENGTH>, JoinedIMUData::numOfInertialDataSources> gyroDataBuffersY;

  bool initialized;
  Vector2f lastFieldSize = Vector2f::Zero();

  PoseHypotheses poseHypotheses;

  Pose2f lastOdometryData;
  bool foundGoodPosition;
  unsigned timeStampLastGoodPosition;
  Pose2f lastGoodPosition;
  Pose2f distanceTraveledFromLastGoodPosition;
  LocalizationState localizationState;
  LocalizationState localizationStateAfterGettingUp;
  unsigned penalizedTimeStamp;
  unsigned unpenalizedTimeStamp;
  unsigned lastPenalty;

  unsigned lastNonPlayingTimeStamp; // needed to prevent too early "positionLost"
  unsigned lastPositionLostTimeStamp; // needed to prevent too early spawning of symmetric positions
  unsigned timeStampFirstReadyState; // needed for preventing spawning on opponent half in first ready state
  float distanceTraveledFromLastFallDown; // needed for preventing spawning when fallen down within a certain range

  unsigned lastNonSetTimestamp;
  RingBuffer<Vector3f, 30> accDataBuffer;
  bool gotPickedUp;


  /* ----------------------------------- private local storage ------------------------------------*/
  RobotPose m_robotPose;
  SideConfidence m_sideConfidence;
  RobotPoseHypothesis m_robotPoseHypothesis;
  RobotPoseHypotheses m_robotPoseHypotheses;

  /* ----------------------------------- private methods here ------------------------------------*/

  void execute(tf::Subflow&);

  void adjustHypothesesToFieldDimensions();
  void checkBeingPickedUp();

  void handleFallDown();
  void handleGettingUpAfterFallDown();
  void handleUnPenalized();
  void handleSetState();
  void handleInitialState();

  void normalizeWeights();
  void pruneHypotheses();
  void pruneHypothesesInOwnHalf();
  void pruneHypothesesInOpponentHalf();
  void pruneHypothesesOutsideField();
  void pruneHypothesesOutsideCarpet();
  void pruneHypothesesWithInvalidValues();
  void predictHypotheses();
  void updateHypothesesPositionConfidence();
  void updateHypothesesState();
  void updateHypothesesSymmetryConfidence();

  bool addNewHypotheses();

  bool addNewHypothesesFromLineMatches(bool onlyAddUnique = false);
  bool addNewHypothesesFromLandmark();
  bool addNewHypothesesFromPenaltyCrossLine();
  bool addNewHypothesesFromCenterCirleAndLine();
  bool addNewHypothesesFromGoal();

  void addHypothesesOnManualPositioningPositions();
  void addHypothesesOnInitialKickoffPositions();
  void addPenaltyStrikerStartingHypothesis();
  void addHypothesesOnInitialPositions(float newPositionConfidence);
  void addHypothesesOnPenaltyPositions(float newPositionConfidence);

  void evaluateLocalizationState();

  bool hasPositionTrackingFailed(); //this is only for the best
  bool hasPositionBeenFoundAfterLoss();

  bool hasSymmetryBeenLost(const PoseHypothesis2017& hypotheses);
  bool hasSymmetryBeenFoundAgainAfterLoss(const PoseHypothesis2017& hypotheses);

  uint64_t lastBestHypothesisUniqueId;

  const PoseHypothesis2017* getBestHypothesis();

  float getRobotPoseValidity(const PoseHypothesis2017& poseHypothesis);

  void addPoseToHypothesisVector(const Pose2f& pose, std::vector<HypothesisBase>& additionalHypotheses, const float& poseConfidence);


  /**
  * all the debugging done for the whole module inside the execute
  */
  void initDebugging();
  void doGlobalDebugging();
  void generateOutputData();
};
