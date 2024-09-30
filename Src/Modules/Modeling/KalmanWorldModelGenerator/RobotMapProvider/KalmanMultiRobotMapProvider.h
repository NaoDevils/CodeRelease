/**
 * \file KalmanMultiRobotMapProvider.h
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 *
 * Declaration of module KalmanMultiRobotMapProvider.
 * This module provides a local, remote and merged robot map by using a multiple kalman filter approach.
 */


#pragma once

// ------------- NAO-Framework includes --------------
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

// Requires
#include "Representations/Infrastructure/FrameInfo.h" // frame timestamp
#include "Representations/Configuration/FieldDimensions.h" // field dimensions used for removing hypotheses outside the field
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/SonarPercept.h" // sonar percept
#include "Representations/Infrastructure/RobotInfo.h" // robot info used for getting player number and penalty info.
#include "Representations/Infrastructure/TeammateData.h" // team mates information
#include "Representations/Modeling/RobotPose.h" // robot pose
#include "Representations/MotionControl/OdometryData.h" // odometry

// - Percepts
#include "Representations/Perception/RobotsPercept.h"

// Provides
#include "Representations/Modeling/RobotMap.h" // RobotMaps

// ------------- KalmanRobotMapProvider includes --------------
#include "../AngleModels/LocalMultiKalmanModelAngle.h"
#include "../AngleModels/RemoteMultiKalmanModelAngle.h"
#include "Models/LocalRobotMapHypothesis.h"
#include "Models/RemoteRobotMapHypothesis.h"
#include "KalmanMultiRobotMapProviderParameters.h"

#define ROBOTPERCEPT_BUFFER_LENGTH 4 * 30 // percept buffer for 10 seconds

MODULE(KalmanMultiRobotMapProvider,
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraMatrixUpper),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SonarPercept),

  REQUIRES(RobotsPercept),
  REQUIRES(TeammateData),

  REQUIRES(OdometryData), // RobotEstimate-Buffer

  PROVIDES(LocalRobotMap), // Local RobotMap
  PROVIDES(RemoteRobotMap), // Remote RobotMap
  PROVIDES(RobotMap), // Complete RobotMap (local+remote)
  HAS_PREEXECUTION,
  LOADS_PARAMETERS(,
    /// Parameters used by \c KalmanMultiRobotMapProvider.
    (KalmanMultiRobotMapProviderParameters) parameters,
    /// Fix kalman filter noise matrices which are used to initialize the kalman
    /// filter of each new hypothesis.
    (KalmanPositionTracking2D<double>::KalmanMatrices::Noise) kalmanNoiseMatrices
  )
);

class KalmanMultiRobotMapProvider : public KalmanMultiRobotMapProviderBase
{
private:
  static constexpr unsigned LOCAL_PERCEPT_DURATION = 4000;
  static constexpr unsigned REMOTE_PERCEPT_DURATION = 10000;

public:
  /**
   * ctor.
   */
  KalmanMultiRobotMapProvider() : m_localKalmanRobotMap(LOCAL_PERCEPT_DURATION), m_remoteKalmanRobotMap(REMOTE_PERCEPT_DURATION), m_mergedKalmanRobotMap(REMOTE_PERCEPT_DURATION)
  {
    reset();
  };

  /**
   * dtor
   */
  ~KalmanMultiRobotMapProvider(){};

  /**
   * This method provides the robot map (local + remote).
   * \param [out] robotMap The robot map (absolute field coordinates).
   */
  void update(RobotMap& robotMap) override;

  /**
   * This method provides the local robot map.
   * \param [out] localRobotMap The local robot map (absolute field coordinates).
   */
  void update(LocalRobotMap& localRobotMap) override;

  /**
   * This method provides the remote robot map.
   * \param [out] remoteRobotMap The remote robot map (absolute field coordinates).
   */
  void update(RemoteRobotMap& remoteRobotMap) override;

private:
  // Local Kalman robot map
  using LocalKalmanRobotMap = GlobalMultiKalmanModelAngle<LocalRobotMapHypothesis, false>;
  // Remote Kalman robot map
  using RemoteKalmanRobotMap = RemoteMultiKalmanModelAngle<RemoteRobotMapHypothesis, false>;
  // Merged Kalman robot map
  using MergedKalmanRobotMap = RemoteMultiKalmanModelAngle<RemoteRobotMapHypothesis, false>;

  /**
 * Save the \c OdometryData at each invocation of the method \c execute().
 * This is used one iteration later to compute the odometry change since the
 * last iteration.
 */
  OdometryData m_lastOdometryData;

  // The robot map (local percepts only)
  LocalRobotMap m_localRobotMap;
  // The robot map (remote percepts only)
  RemoteRobotMap m_remoteRobotMap;
  // The robot map (merged)
  RobotMap m_robotMap;

  // The kalman map for local percepts
  LocalKalmanRobotMap m_localKalmanRobotMap;
  // The kalman map for remote percepts
  RemoteKalmanRobotMap m_remoteKalmanRobotMap;
  // The kalman map for merged percepts
  MergedKalmanRobotMap m_mergedKalmanRobotMap;

  RingBuffer<std::vector<RobotEstimate>, ROBOTPERCEPT_BUFFER_LENGTH> perceptBuffer;


  // Reset the provider
  void reset();

  // EXECUTE MODULE
  // Execute the code
  void execute(tf::Subflow&) override;
  // Motion update
  void motionUpdate();
  // Sensor update
  void sensorUpdate();
  // Prune hypotheses
  void pruneHypotheses();
  // Update robot types
  void updateRobotTypes();
  // Generate maps
  void generateMaps();

  // DEBUGGING
  // Init modifies
  void modifyInternalParameters();
  // Init Drawings
  void initDebugging();
  // Draw everything
  void doGlobalDebugging();

  // HELPERS
  // Update validities
  void updateValidities();

  void getAngles(Vector2a& angles, const Vector2f& relativePosition);

  // Merge hypotheses
  template <typename RobotMap> void mergeHypotheses(RobotMap& robotMap, const Vector2a& mergeAngleDiff, const float mergeDistance);

  // Find nearest pose
  template <typename RobotMap>
  typename RobotMap::HypothesisType* findNearestHypothesis(RobotMap& robotMap, Vector2a& angleDiff, const Vector2f& measuredPosition, float& distance, RobotEstimate::RobotType desiredRobotType);

  // Update sensor
  template <template <typename T> class SensorUpdateArgs, typename RobotMap> typename RobotMap::HypothesisType* performSensorUpdate(const SensorUpdateArgs<RobotMap>& args);

  // Update sensor
  template <template <typename T> class SensorUpdateArgs, typename RobotMap>
  void performSensorUpdate(const SensorUpdateArgs<RobotMap>& args, int playerNumber, const typename RemoteRobotMapHypothesis::TeammateInfo& teammate);
};
