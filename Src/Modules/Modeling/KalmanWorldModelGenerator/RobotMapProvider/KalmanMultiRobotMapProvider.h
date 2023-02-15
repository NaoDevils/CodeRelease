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

// Requires
#include "Representations/Infrastructure/FrameInfo.h" // frame timestamp
#include "Representations/Configuration/FieldDimensions.h" // field dimensions used for removing hypotheses outside the field
#include "Representations/Infrastructure/RobotInfo.h" // robot info used for getting player number and penalty info.
#include "Representations/Infrastructure/TeammateData.h" // team mates information
#include "Representations/Modeling/RobotPose.h" // robot pose

// - Percepts
#include "Representations/Perception/RobotsPercept.h"

// Provides
#include "Representations/Modeling/RobotMap.h" // RobotMaps

// ------------- KalmanRobotMapProvider includes --------------
#include "../Models/LocalMultiKalmanModel.h"
#include "../Models/RemoteMultiKalmanModel.h"
#include "Models/LocalRobotMapHypothesis.h"
#include "Models/RemoteRobotMapHypothesis.h"
#include "KalmanMultiRobotMapProviderParameters.h"


MODULE(KalmanMultiRobotMapProvider,
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),

  REQUIRES(RobotsPercept),
  REQUIRES(RobotsPerceptUpper),
  REQUIRES(TeammateData),

  PROVIDES(LocalRobotMap), // Local RobotMap
  PROVIDES(RemoteRobotMap), // Remote RobotMap
  PROVIDES(RobotMap), // Complete RobotMap (local+remote)

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
  static constexpr unsigned LOCAL_PERCEPT_DURATION = 1000;
  static constexpr unsigned REMOTE_PERCEPT_DURATION = 4000;

public:
  /**
   * ctor.
   */
  KalmanMultiRobotMapProvider()
      : m_lastTimeStamp(0), m_localKalmanRobotMap(LOCAL_PERCEPT_DURATION), m_remoteKalmanRobotMap(REMOTE_PERCEPT_DURATION), m_mergedKalmanRobotMap(REMOTE_PERCEPT_DURATION)
  {
    initialize();
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
  using LocalKalmanRobotMap = GlobalMultiKalmanModel<LocalRobotMapHypothesis, false>;
  // Remote Kalman robot map
  using RemoteKalmanRobotMap = RemoteMultiKalmanModel<RemoteRobotMapHypothesis, false>;
  // Merged Kalman robot map
  using MergedKalmanRobotMap = RemoteMultiKalmanModel<RemoteRobotMapHypothesis, false>;

  // last execution timestamp
  unsigned m_lastTimeStamp;

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

  // Initialize the provider
  void initialize();

  // EXECUTE MODULE
  // Execute the code
  void execute();
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
  // Merge hypotheses
  template <typename RobotMap> void mergeHypotheses(RobotMap& robotMap)
  {
    // Iterate over hypotheses
    for (unsigned i = 0; i < robotMap.size(); i++)
    {
      auto& iRobot = robotMap[i];
      for (unsigned j = i + 1; j < robotMap.size(); j++)
      {
        auto& jRobot = robotMap[j];

        // Get distance between measurements
        float distance = Geometry::distance(iRobot.kalman.position(), jRobot.kalman.position());
        // Get differene in robot type
        float robotTypeDifference = std::abs(iRobot.robotTypeValidity() - jRobot.robotTypeValidity());

        // Check whether hypotheses close enough for merging
        if (distance < parameters.localMapParameters.maxDistanceToMerge && robotTypeDifference < parameters.localMapParameters.maxRobotTypeDifferenceToMerge)
        {
          // Merge
          iRobot.merge(jRobot);
          // Set validity of the other to 0 to prune it later
          jRobot.validity = 0;
        }
      }
    }
  }
  // Find nearest pose
  template <typename RobotMap>
  typename RobotMap::HypothesisType* findNearestHypothesis(RobotMap& robotMap, const Vector2f& measuredPosition, float& distance, RobotEstimate::RobotType desiredRobotType)
  {
    typename RobotMap::HypothesisType* nearestHypothesis = nullptr;
    float minDistance = -1;

    for (size_t i = 0; i < robotMap.size(); i++)
    {
      // Only consider hypotheses with the correct robot type.
      if (!robotMap[i].matchRobotType(desiredRobotType))
        continue;

      // Calculate distance from the perception to the current hypothesis.
      float currentDistance = Geometry::distance(measuredPosition, robotMap[i].kalman.position());

      // Check whether the current distance is shorter than the minimum of all
      // previous hypotheses.
      if (minDistance < 0 || currentDistance < minDistance)
      {
        nearestHypothesis = &robotMap[i];
        minDistance = currentDistance;
      }
    }

    distance = minDistance;
    return nearestHypothesis;
  }
  // Update sensor
  template <typename RobotMap>
  typename RobotMap::HypothesisType* performSensorUpdate(RobotMap& robotMap,
      const Vector2f& measuredPosition,
      float measuredDistance,
      const Vector2f* measuredVelocity,
      RobotEstimate::RobotType desiredRobotType,
      unsigned timestamp,
      float perceptValidity,
      float minDistanceForNewHypothesis,
      float initialValidityForNewHypothesis,
      const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices)
  {
    if (perceptValidity <= 0.f)
      return nullptr; // Check whether percept validity is positive before performing update

    // Need to re-implement this method as we want to take robot type into account when searching for nearest hypothesis to update

    float distance;
    typename RobotMap::HypothesisType* nearestHypothesis = findNearestHypothesis(robotMap, measuredPosition, distance, desiredRobotType);
    if (nearestHypothesis != nullptr && distance < minDistanceForNewHypothesis)
    {
      // TODO: make parameter
      float measurementNoiseFactor = measuredDistance / 1000.f < 1.f ? 1.f : measuredDistance / 1000.f; // convert distance to m
      // Set measurementNoiseMatrix according to the direction from robot to measurement position.
      // In this direction (^= distance) the noise is set to a higher value than in the orthogonal direction (^= angle).
      float max = static_cast<float>(nearestHypothesis->kalman.matrices.noise.maxMeasurementNoise);
      max *= measurementNoiseFactor;
      nearestHypothesis->kalman.matrices.noise.measurementNoiseMatrix = Covariance::create((Vector2f() << max, max / 10.f).finished(), measuredPosition.angle()).cast<double>();

      // Correct kalman filter with the perception.
      if (measuredVelocity == nullptr)
        nearestHypothesis->sensorUpdate(measuredPosition, timestamp, perceptValidity);
      else
        nearestHypothesis->sensorUpdate(measuredPosition, *measuredVelocity, timestamp, perceptValidity);
    }
    else
    {
      bool isLocalMap = static_cast<void*>(&robotMap) == static_cast<void*>(&m_localKalmanRobotMap);
      // Add new hypothesis if all existing ones are too far.
      typename RobotMap::HypothesisType newHypothesis(
          kalmanNoiseMatrices, initialValidityForNewHypothesis, timestamp, perceptValidity, measuredPosition, isLocalMap ? LOCAL_PERCEPT_DURATION : REMOTE_PERCEPT_DURATION);

      robotMap.addHypothesis(std::move(newHypothesis));
      nearestHypothesis = &robotMap.back();
    }

    // Update robot type sensor
    nearestHypothesis->addPerceptRobotType(desiredRobotType, timestamp);

    return nearestHypothesis;
  }
  // Update sensor
  template <typename RobotMap>
  void performSensorUpdate(RobotMap& robotMap,
      const Vector2f& measuredPosition,
      float measuredDistance,
      const Vector2f* measuredVelocity,
      RobotEstimate::RobotType desiredRobotType,
      unsigned timestamp,
      float perceptValidity,
      float minDistanceForNewHypothesis,
      float initialValidityForNewHypothesis,
      const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      int playerNumber,
      const typename RemoteRobotMapHypothesis::TeammateInfo& teammate)
  {
    typename RobotMap::HypothesisType* nearestHypothesis = performSensorUpdate(
        robotMap, measuredPosition, measuredDistance, measuredVelocity, desiredRobotType, timestamp, perceptValidity, minDistanceForNewHypothesis, initialValidityForNewHypothesis, kalmanNoiseMatrices);

    // Add a teammate label to the hypothesis.
    if (nearestHypothesis) // Check that there is a hypothesis (handle perceptValidity <= 0 case)
      nearestHypothesis->addTeammateInfo(playerNumber, teammate);
  }
};
