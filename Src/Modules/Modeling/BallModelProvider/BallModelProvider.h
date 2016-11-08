/**
 * \file BallModelProvider.h
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
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
#include "Representations/Infrastructure/RobotInfo.h" // robot info used for getting player number and penalty info.
#include "Representations/Infrastructure/TeammateData.h" // team mates information

// - Percepts
#include "Representations/Perception/BallPercept.h"

// Uses
#include "Representations/Modeling/RobotPose.h" // robot pose (warning: it's the pose from last frame)

// Provides
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Modeling/TeamBallModel.h"

// ------------- BallModelProvider includes --------------
#include "TeamBallModelProvider.h"
#include "BallModelProviderParameters.h"
#include "TeamBallModelParameters.h"
#include "Models/MultipleBallModel.h"
#include "Tools/Math/KalmanMultiDimensional.h"


MODULE(BallModelProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OdometryData),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(RobotInfo),

  REQUIRES(BallPercept),
  REQUIRES(TeammateData),

  //REQUIRES(GroundTruthBallModel), // Compare with GroundTruthBallModel for evaluation.

  USES(RobotPose), // Warning: it's the pose from last iteration

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
    /// Fix kalman filter matrices which are used to initialize the kalman filter of
    /// each new ball hypothesis.
    (KalmanPositionTracking2D<double>::KalmanMatrices) kalmanMatrices,
  }), 
});


class BallModelProvider : public BallModelProviderBase
{
public:

  /** 
   * Constructor.
   */
  BallModelProvider() : m_lastTimeStamp(0),
                        m_lastGameState(STATE_INITIAL),
                        m_lastPenaltyState(PENALTY_NONE),
                        m_ballDisappeared(true),
                        m_timeWhenBallFirstDisappeared(0)
  {
    initialize();
  }

  /** 
   * Destructor.
   */
  ~BallModelProvider() {}
  
  
  // Update methods for provided representations.

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

  // BallModelProvider methods.

  /**
   * Initializes the \c BallModelProvider and all provided ball models.
   */
  void initialize();

  /**
   * \brief Executes everything which is necessary to update the provided representations.
   *
   * This method is used to update the state of the self locator only once per
   * frame. Therefor the execution timestamp is saved. On later exectutions the
   * timestamp is checked in order to determine whether this method was already
   * executed in the current frame.
   */
  void execute();

  /**
   * \brief Prediction
   *
   * This method performs the prediction step of each ball hypothesis.
   * \param [in] timeOffset The time in seconds since last iteration.
   */
  void motionUpdate(float timeOffset);

  /**
   * \brief Correction
   *
   * This method performs the correction step of each ball hypothesis.
   * \param [in] timeOffset The time in seconds since last iteration.
   */
  void sensorUpdate(float timeOffset);

  /**
   * \brief Correction of \c m_localMultipleBallModel
   * This method performs the correction step of each local ball hypothesis.
   * \param [in] timeOffset The time in seconds since last iteration.
   */
  void sensorUpdateLocal(float timeOffset);

  /**
   * \brief Correction of \c m_remoteMultipleBallModel
   * This method performs the correction step of each remote ball hypothesis.
   * \param [in] timeOffset The time in seconds since last iteration.
   */
  void sensorUpdateRemote(float timeOffset);
  
  /**
   * This method executes game state depending actions.
   */
  void handleGameState();
  
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
  void updateTimeWhenBallFirstDisappeared(const BallHypothesis* hypothesis, BallModel& ballModel);
  
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


  // Debug methods.

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
  void draw() const;
  
  
private:
  // ----- local ball model -----
  /**
   * The local \c BallModel which is provided by this module (local robot coordinates).
   */
  BallModel m_localBallModel;
  /**
   * The storage of all local ball hypotheses (local robot coordinates).
   */
  MultipleBallModel m_localMultipleBallModel;
  
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
   * Save state of penalty at each invocation of the method \c handleGameState().
   * This is used to remove all ball hypotheses after a penalty.
   */
  uint8_t m_lastPenaltyState;
  
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
