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

#include <algorithm>

// ------------- NAO-Framework includes --------------
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Module.h"
#include "Tools/ColorClasses.h"
#include "Representations/Modeling/RobotMap.h"
#include "Representations/Modeling/RobotPoseHypotheses.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/LineMatchingResult.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/CenterCirclePercept.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/PenaltyCrossPercept.h"
#include "Representations/Perception/CLIPFieldLinesPercept.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/BehaviorControl/BehaviorData.h"

#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/GaussianDistribution3D.h"

#include "Tools/Modeling/PoseGenerator.h"

// ------------- World Model Generator includes --------------
#include "models/PoseHypothesis2017.h"
#include "models/PoseHypotheses2017.h"

#include "SelfLocator2017Parameters.h"

MODULE(SelfLocator2017,
{ ,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(FrameInfo),
  REQUIRES(TeammateData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OdometryData),
  REQUIRES(LineMatchingResult),
  REQUIRES(CLIPCenterCirclePercept),
  REQUIRES(CLIPGoalPercept),
  REQUIRES(PenaltyCrossPercept),
  REQUIRES(BallModel),
  USES(RemoteBallModel),
  USES(LocalRobotMap),
  USES(RemoteRobotMap),
  REQUIRES(FallDownState),
  REQUIRES(CLIPFieldLinesPercept),
  REQUIRES(JointSensorData),
  USES(BehaviorData), // For manual positions, role is required (right now only goalie vs field player)
  PROVIDES(RobotPose),
  PROVIDES_WITHOUT_MODIFY(RobotPoseHypotheses), //all
  PROVIDES_WITHOUT_MODIFY(RobotPoseHypothesis), //the best
  PROVIDES(SideConfidence),
  LOADS_PARAMETERS(
  {,
    (PositionsByRules) positionsByRules,
    (SelfLocator2017Parameters) parameters,
  }),
});




class SelfLocator2017 : public SelfLocator2017Base
{
  struct HypothesesBase
  {
    Pose2f pose;
    float positionConfidence;
    double symmetryConfidence;

    HypothesesBase(const Pose2f &_pose, float _positionConfidence, double _symmetryConfidence)
      : pose(_pose), positionConfidence(_positionConfidence), symmetryConfidence(_symmetryConfidence) { }
  };

  ENUM(LocalizationState,
  { ,
    positionTracking,
    fallenDown,
    positionLost,
    penalized,
  })

public:

  /*------------------------------ public methods -----------------------------------*/

  /** 
  * Constructor.
  */
  SelfLocator2017(); 

  /** 
  * Destructor.
  */
  ~SelfLocator2017();

  /**
  * The function executes the module.
  */
  void update(RobotPose& robotPose);
  void update(RobotPoseHypothesis& robotPoseHypothesis);
  void update(RobotPoseHypotheses& robotPoseHypotheses);
  void update(SideConfidence& sideConfidence);
private:
  /* ----------------------------------- private variables here ----------------------------------*/

  bool initialized;
  unsigned lastExecuteTimeStamp;
  
  PoseHypotheses2017 poseHypotheses;
  
  Pose2f                    lastOdometryData;
  bool                      foundGoodPosition;
  bool                      symmetryLost;
  unsigned                  timeStampLastGoodPosition;
  Pose2f                    lastGoodPosition;
  Pose2f                    distanceTraveledFromLastGoodPosition;
  LocalizationState         localizationState;
  LocalizationState         localizationStateAfterGettingUp;
  unsigned                  penalizedTimeStamp;
  unsigned                  unpenalizedTimeStamp;
  unsigned                  lastPenalty;

  unsigned                  lastNonPlayingTimeStamp; // needed to prevent too early "positionLost"
  unsigned                  lastPositionLostTimeStamp; // needed to prevent too early spawning of symmetric positions
  unsigned                  timeStampFirstReadyState; // needed for preventing spawning on opponent half in first ready state

  /* ----------------------------------- private methods here ------------------------------------*/

  void executeCommonCode();

  void handleFallDown();
  void handleGettingUpAfterFallDown();
  void handleUnPenalized();
  void handleSetState();
  void handleInitialState();

  void normalizeWeights();
  void pruneHypotheses();
  void pruneHypothesesGoalie();
  void pruneHypothesesInOwnHalf();
  void pruneHypothesesOutsideCarpet();
  void pruneHypothesesWithInvalidValues();
  void predictHypotheses();
  void updateHypothesesPositionConfidence();
  void updateHypothesesState();
  void updateHypothesesSymmetryConfidence();
  
  bool addNewHypotheses();
  
  bool addNewHypothesesWhenSymmetryLost();

  bool addNewHypothesesFromLineMatches();
  bool addNewHypothesesFromLandmark();
  bool addNewHypothesesFromPenaltyCrossLine();
  bool addNewHypothesesFromCenterCirleAndLine();
  bool addNewHypothesesFromGoal();
  
  void addHypothesesOnManualPositioningPositions();
  void addHypothesesOnInitialKickoffPositions();
  void addHypothesesOnPenaltyPositions(float newPositionConfidence);

  void evaluateLocalizationState();

  bool hasPositionTrackingFailed(); //this is only for the best 
  bool hasPositionBeenFoundAfterLoss();

  bool hasSymmetryBeenLostForBestHypotheses();
  bool hasSymmetryBeenLost(const PoseHypothesis2017& hypotheses);

  bool hasSymmetryBeenFoundAgainAfterLoss();
  bool hasSymmetryBeenFoundAgainAfterLoss(const PoseHypothesis2017& hypotheses);

  uint64_t lastBestHypothesisUniqueId;
  
  const PoseHypothesis2017 &getBestHypothesis();

  float getRobotPoseValidity(const PoseHypothesis2017 & poseHypothesis);

  void addPoseToHypothesisVector(const Pose2f &pose, std::vector<HypothesesBase> &additionalHypotheses,
    const float &poseConfidence);



  /**
  * all the debugging done for the whole module inside the execute
  */
  void initDebugging();
  void doGlobalDebugging();
};
