/**
 * \file BallModelProvider.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Declaration of module BallModelProvider.
 * This module provides both the local and the remote ball model. Multiple 
 * ball hypotheses are possible.
 */

#pragma once


// ------------- NAO-Framework includes --------------
#include "Tools/Module/Module.h"

// Requires
#include "Representations/Infrastructure/FrameInfo.h" // frame timestamp
#include "Representations/Infrastructure/GameInfo.h" // game state information
#include "Representations/MotionControl/OdometryData.h" // odometry
#include "Representations/Configuration/FieldDimensions.h" // field dimensions used for removing hypotheses outside the field
#include "Representations/Infrastructure/CameraInfo.h" // camera info used for checking if ball should be visible (ballLost?)
#include "Representations/Perception/CameraMatrix.h" // camera matrix used for checking if ball should be visible (ballLost?)
#include "Representations/Infrastructure/TeamInfo.h" // team info used for getting info about kicking team
#include "Representations/Infrastructure/RobotInfo.h" // robot info used for getting player number and penalty info.
#include "Representations/Infrastructure/TeammateData.h" // team mates information

// - Percepts
#include "Representations/Perception/BallPercept.h"

// Uses
#include "Representations/Modeling/RobotPose.h" // robot pose
#include "Representations/MotionControl/MotionInfo.h" // motion info (walk, kick, etc.)

// Provides
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/TeamBallModel.h"

// ------------- BallModelProvider includes --------------
#include "TeamBallModelProvider.h"
#include "Models/MultipleBallModel.h"
#include "BallModelProviderParameters.h"
#include "TeamBallModelParameters.h"


MODULE(BallModelProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OdometryData),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),

  REQUIRES(BallPercept), // Use either single or multiple ball percept.
  REQUIRES(MultipleBallPercept), // Multiple ball percept is used when MultipleBallPercept::balls is not empty.
  REQUIRES(TeammateData),

  //REQUIRES(GroundTruthBallModel), // Compare with GroundTruthBallModel for evaluation.

  USES(RobotPose), // Warning: it's the pose from last iteration
  USES(MotionInfo), // Warning: it's the motion info from last iteration

  PROVIDES(BallModel),
  PROVIDES(RemoteBallModel),
  PROVIDES(TeamBallModel),

  LOADS_PARAMETERS(
  {,
    /// Parameters used by \c BallModelProvider for local and remote ball model.
    (BallModelProviderParameters) parameters,
    /// Parameters used by \c TeamBallModelProvider for merging local and remote
    /// ball models to team ball model.
    (TeamBallModelParameters) teamParameters,
    /// Fix kalman filter noise matrices which are used to initialize the kalman
    /// filter of each new ball hypothesis.
    (KalmanPositionTracking2D<double>::KalmanMatrices::Noise) kalmanNoiseMatrices,
  }), 
});


class BallModelProvider : public BallModelProviderBase
{
private:
  static constexpr unsigned LOCAL_PERCEPT_DURATION = 1000;
  static constexpr unsigned REMOTE_PERCEPT_DURATION = 4000;

public:

  /** 
   * Constructor.
   */
  BallModelProvider()
    : m_localMultipleBallModel(LOCAL_PERCEPT_DURATION)
    , m_remoteMultipleBallModel(REMOTE_PERCEPT_DURATION)
    , m_lastTimeStamp(0)
    , m_lastGameState(STATE_INITIAL)
    , m_lastPenaltyState(PENALTY_NONE)
    , m_lastMotionType(MotionRequest::Motion::specialAction)
    , m_kickDetected(false)
    , m_ballDisappeared(true)
    , m_timeWhenBallFirstDisappeared(0)
  {
    initialize();
  }

  /** 
   * Destructor.
   */
  ~BallModelProvider() {}
  
  
  // MARK: Update methods for provided representations.

  /**
   * This method provides the local ball model. It is computes only by own
   * percepts.
   * \param [out] ballModel The local ball model (relative robot coordinates).
   */
  void update(BallModel& ballModel);

  /**
   * This method provides the remote ball model. It is computes only by ball 
   * models from teammates.
   * \param [out] remoteBallModel The remote ball model (absolute field coordinates).
   */
  void update(RemoteBallModel& remoteBallModel);
  
  /**
   * This method provides the team ball model. It is merged from both the local
   * and the remote ball model. The remote ball model is derived from the local
   * ball models of team mates which are received via the \c TeammateData.
   * \param [out] teamBallModel The team ball model (absolute field coordinates).
   */
  void update(TeamBallModel& teamBallModel);
  

private:
  // MARK: BallModelProvider methods.

  /**
   * Initializes the \c BallModelProvider and all provided ball models.
   */
  void initialize();

  /**
   * \brief Executes everything which is necessary to update the provided representations.
   *
   * This method is used to update the state of this module only once per
   * frame. Therefor the execution timestamp is saved. On later exectutions the
   * timestamp is checked in order to determine whether this method was already
   * executed in the current frame.
   */
  void execute();

  /**
   * \brief Prediction
   *
   * This method performs the prediction step of each ball hypothesis.
   */
  void motionUpdate();

  /**
   * \brief Correction
   *
   * This method performs the correction step of each ball hypothesis.
   */
  void sensorUpdate();

  /**
   * \brief Correction of \c m_localMultipleBallModel
   *
   * This method performs the correction step of each local ball hypothesis.
   */
  void sensorUpdateLocal();
  
  /**
   * \brief Correction of \c m_localMultipleBallModel with one percept
   *
   * This method performs the correction step of each local ball hypothesis with
   * the given ball percept.
   * This method is used in \c sensorUpdateLocal to reduce code duplication.
   * \param [in] ball The ball percept.
   */
  void sensorUpdateLocalSingle(const BallPercept& ball);

  /**
   * \brief Correction of \c m_remoteMultipleBallModel
   *
   * This method performs the correction step of each remote ball hypothesis.
   */
  void sensorUpdateRemote();
  
  /**
   * This method executes game state depending actions.
   */
  void handleGameState();
  
  /**
   * This method checks if a kick is executed and increases the validity 
   * uncertainty.
   */
  void handleKick();
  
  /**
   * \brief Checks whether the ball is disappeared or not.
   * 
   * The ball is believed to be disappeared, if it should be visible in current
   * image by any of the two cameras, but is not. The timestamp 
   * \c timeWhenBallFirstDisappeared contains the first time when the ball was 
   * disappeared or the current time, if the ball is not disappeared. Accordingly 
   * to this rule the timestamp is updated by this method and the member variable 
   * \c ballDisappeared is set to the disappeared state.
   * \param [in] hypothesis Use this hypothesis for checking whether the ball 
   *                        should be visible or not.
   * \param [in, out] ballModel ballModel.timeWhenDisappeared is updated by this method.
   * \see ballDisappeared, timeWhenBallFirstDisappeared, BallModel::timeWhenDisappeared
   */
  void updateTimeWhenBallFirstDisappeared(const KalmanPositionHypothesis* hypothesis, BallModel& ballModel);
  
  /**
   * Fills \c m_localBallModel with contents of \c m_localMultipleBallModel 
   * (best hypothesis).
   */
  void generateLocalBallModel();

  /**
   * Fills \c m_remoteBallModel with contents of \c m_remoteMultipleBallModel 
   * (best hypothesis).
   */
  void generateRemoteBallModel();


  // MARK: Debug methods.

  /**
   * Allows modification of some parameters in SimRobot.
   */
  void initModify();

  /**
   * Declares debug drawings for SimRobot.
   */
  void initDebugDrawing() const;

  /**
   * Does debug drawings.
   */
  void draw();
  
  
private:
  // ----- local ball model -----
  /**
   * The local \c BallModel which is provided by this module (local robot coordinates).
   */
  BallModel m_localBallModel;
  /**
   * The storage of all local ball hypotheses (local robot coordinates).
   */
  LocalMultipleBallModel m_localMultipleBallModel;
  
  // ----- remote ball model (only teammates) -----
  /**
   * The \c RemoteBallModel which is provided by this module (global field coordinates).
   */
  RemoteBallModel m_remoteBallModel;
  /**
   * The storage of all remote ball hypotheses (global field coordinates).
   */
  RemoteMultipleBallModel m_remoteMultipleBallModel;

  /**
   * Saves the timestamp of the last percept (from the robot with player number
   * given by the vector index) which was used to update the multiple team ball
   * model.
   * This assures that each percept is used only once for sensor update.
   */
  std::vector<unsigned> m_remoteTimeSinceBallSeen;

  // ----- team ball model (merged from local and remote ball models) -----
  /**
   * The \c TeamBallModelProvider handles the merging of local and remote ball 
   * models. It is not a single module, but a part of the module \c BallModelProvider, 
   * because it needs access to the local and remote ball models from this module.
   */
  TeamBallModelProvider m_teamBallModelProvider;

  // ----- general variables -----
  /**
   * Save timestamp when executing the method \c execute(), to run it only once
   * per frame.
   */
  unsigned m_lastTimeStamp;
  
  /**
   * Save game state at each invocation of the method \c handleGameState(). This
   * is used to handle state transitions.
   */
  uint8_t m_lastGameState;

  /**
   * Save set play state at each invocation of the method \c handleGameState(). This
   * is used to handle state transitions.
   */
  uint8_t m_lastSetPlay;
  
  /**
   * Save state of penalty at each invocation of the method \c handleGameState().
   * This is used to remove all ball hypotheses after a penalty.
   */
  uint8_t m_lastPenaltyState;
  
  /**
   * Save motion type at each invocation of the method \c handleKick(). This is 
   * used to check if a new kick is triggered.
   */
  MotionRequest::Motion m_lastMotionType;
  /**
   * This is set to \c true by \c handleKick() while \c MotionRequest::motion is 
   * \c true and the ball velocity was increased in at least one frame.
   * \see m_lastMotionType
   */
  bool m_kickDetected;
  
  /**
   * Save the \c OdometryData at each invocation of the method \c execute().
   * This is used one iteration later to compute the odometry change since the
   * last iteration.
   */
  OdometryData m_lastOdometryData;
  
  /**
   * The ball is believed to be disappeared, if it should be visible in current
   * image by any of the two cameras, but is not.
   * \see timeWhenBallFirstDisappeared
   */
  bool m_ballDisappeared;
  /**
   * The ball is believed to be disappeared, if it should be visible in current 
   * image by any of the two cameras, but is not.
   * This timestamp contains the first time when the ball was disappeared or the
   * current time, if the ball is not disappeared.
   * \see ballDisappeared
   */
  unsigned m_timeWhenBallFirstDisappeared;
};
