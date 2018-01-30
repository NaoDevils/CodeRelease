/**
 * \file KalmanRobotMapProvider.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Declaration of module KalmanRobotMapProvider.
 * This module provides a remote robot map by using a multiple kalman filter approach.
 */

#pragma once


// ------------- NAO-Framework includes --------------
#include "Tools/Module/Module.h"

// Requires
#include "Representations/Infrastructure/FrameInfo.h" // frame timestamp
#include "Representations/Configuration/FieldDimensions.h" // field dimensions used for removing hypotheses outside the field
#include "Representations/Infrastructure/RobotInfo.h" // robot info used for getting player number and penalty info.
#include "Representations/Infrastructure/TeammateData.h" // team mates information
#include "Representations/Modeling/RobotPose.h" // robot pose

// - Percepts
#include "Representations/Perception/RobotsPercept.h"

// Provides
#include "Representations/Modeling/RobotMap.h" // RemoteRobotMap

// ------------- KalmanRobotMapProvider includes --------------
#include "Models/MultiKalmanRobotMap.h"
#include "KalmanRobotMapProviderParameters.h"


MODULE(KalmanRobotMapProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),

  REQUIRES(RobotsPercept),
  REQUIRES(TeammateData),

  PROVIDES(RobotMap), // Complete RobotMap (local+remote)
  PROVIDES(RemoteRobotMap), // Complete RobotMap (local+remote)

  LOADS_PARAMETERS(
  {,
    /// Parameters used by \c KalmanRobotMapProvider.
    (KalmanRobotMapProviderParameters) parameters,
    /// Fix kalman filter noise matrices which are used to initialize the kalman
    /// filter of each new hypothesis.
    (KalmanPositionTracking2D<double>::KalmanMatrices::Noise) kalmanNoiseMatrices,
  }), 
});


class KalmanRobotMapProvider : public KalmanRobotMapProviderBase
{
public:

  /** 
   * Constructor.
   */
  KalmanRobotMapProvider() : m_lastTimeStamp(0)
  {
    initialize();
  }

  /** 
   * Destructor.
   */
  ~KalmanRobotMapProvider() {}
  
  
  // MARK: Update methods for provided representations

  /**
   * This method provides the robot map (same as remote robot map).
   * \param [out] robotMap The robot map (absolute field coordinates).
   */
  void update(RobotMap& robotMap);
  
  /**
   * This method provides the remote robot map.
   * \param [out] remoteRobotMap The remote robot map (absolute field coordinates).
   */
  void update(RemoteRobotMap& remoteRobotMap);

private:

  // MARK: KalmanRobotMapProvider methods

  /**
   * Initializes the \c KalmanRobotMapProvider and the provided robot map.
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
   * This method performs the prediction step of each kalman hypothesis.
   */
  void motionUpdate();

  /**
   * \brief Correction
   *
   * This method performs the correction step of each kalman hypothesis.
   */
  void sensorUpdate();
  
  /**
   * \brief Correction using the teammates estimated \c RobotPose
   *
   * This method performs the correction step with the estimated \c RobotPose
   * of all teammates.
   */
  void sensorUpdateTeammates();
  
  /**
   * \brief Correction
   *
   * This method performs the correction step with all local robot percepts.
   */
  void sensorUpdateLocal();
  
  /**
   * \brief Correction
   *
   * This method performs the correction step with all remote robot percepts.
   */
  void sensorUpdateRemote();
  
  /**
   * \brief Correction for one robot
   *
   * This method performs the correction step with one robot percept.
   * \param [in] position Position of the robot percept (in global field coordinates).
   * \TODO Only the position of the robot percept is filtered here.
   *       This is still correct now (2017) since the rotation is not filled by the robots perceptor.
   * \param [in] distanceToRobot Distance on field betreen the origin of the
   *                             perception (teammate or self) and the robot percept.
   * \param [in] robotType \c RobotType of the robot percept.
   * \param [in] timeStamp Time of the robot percept.
   * \param [in] perceptValidity The validity of the robot percept.
   * \param [in] playerNumber The number of the teammate (or self) who saw the robot.
   */
  void sensorUpdateSingle(const Vector2f& position,
                          float distanceToRobot,
                          RobotEstimate::RobotType robotType,
                          unsigned timeStamp,
                          float perceptValidity,
                          int playerNumber);

  /**
   * Fills \c m_remoteRobotMap with contents of \c m_remoteMultiKalmanRobotMap.
   */
  void generateRemoteRobotMap();


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
  
  /**
   * Plot hypothesis validities.
   */
  void plot() const;
  
private:
  /**
   * The \c RobotMap which is provided by this module (global field coordinates).
   */
  RobotMap m_robotMap;
  /**
   * The storage of all robot map hypotheses (global field coordinates).
   */
  MultiKalmanRobotMap m_multiKalmanRobotMap;

  // ----- general variables -----
  /**
   * Save timestamp when executing the method \c execute(), to run it only once
   * per frame.
   */
  unsigned m_lastTimeStamp;
};
