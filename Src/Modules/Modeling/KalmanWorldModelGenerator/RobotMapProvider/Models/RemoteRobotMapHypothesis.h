/**
 * \file RemoteRobotMapHypothesis.h
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 * 
 * Declaration of class \c RemoteRobotMapHypothesis which specializes the
 * \c RemoteMultiKalmanPositionHypothesis for robot map modeling.
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/PerceptsPerSecond.h"
#include "Representations/Modeling/RobotMap.h"
#include "RobotTypeHypothesis.h"
#include "../../Models/RemoteKalmanPositionHypothesis.h"


/**
 * \class RemoteRobotMapHypothesis
 */
class RemoteRobotMapHypothesis : public RemoteKalmanPositionHypothesis, public RobotTypeHypothesis
{
public:
  RemoteRobotMapHypothesis() : RemoteKalmanPositionHypothesis(), RobotTypeHypothesis(){};

  /**
   * \brief Constructor with initialization.
   *
   * Creates a new \c RemoteRobotMapHypothesis object with a given initial
   * position and an optional initial velocity. Parameterizes the kalman filter
   * with noise matrices.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter.
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The timestamp of the initial position measurement (in ms).
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] position The initial position (in mm).
   * \param [in] perceptDuration The duration for the PerceptPerSecond buffer.
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown.
   */
  RemoteRobotMapHypothesis(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      float initialValidity,
      unsigned timestamp,
      float perceptValidity,
      const Vector2f& position,
      unsigned perceptDuration,
      const Vector2f& velocity = Vector2f::Zero())
      : RemoteKalmanPositionHypothesis(kalmanNoiseMatrices, initialValidity, timestamp, perceptValidity, position, perceptDuration, velocity), RobotTypeHypothesis(perceptDuration){};

  /**
   * Destructor.
   */
  ~RemoteRobotMapHypothesis() override{};

  /**
   * Updates the given \c RobotMapEntry from this \c RemoteRobotMapHypothesis. The robot map
   * entry object is used in \c RobotMap::robots to represent one robot on the robot map.
   * \param [out] robot The \c RobotMapEntry which should be updated.
   */
  void updateRobotMapEntry(RobotMapEntry& robot) const
  {
    // Set velocity
    robot.velocity = kalman.velocity();
    // Fill pose
    robot.pose.translation = kalman.position(); // (in mm)
    // Get rotation of robot from its motion direction.
    // TODO: This is not the direction the robot is facing
    robot.pose.rotation = robot.velocity.angle(); // (in radiant)
    // Set robot type
    robot.robotType = lastValidRobotType();
  }

  /**
   * Extracts a \c RobotMapEntry from this \c RemoteRobotMapHypothesis. The robot map
   * entry object is used in \c RobotMap::robots to represent one robot on the robot map.
   * \param [out] robot The \c RobotMapEntry which should be updated.
   */
  RobotMapEntry getRobotMapEntry() const
  {
    RobotMapEntry entry;
    updateRobotMapEntry(entry);
    return entry;
  }

  // Fixes "hidden overloaded function" warning
  using RemoteKalmanPositionHypothesis::merge;
  using RobotTypeHypothesis::merge;

  virtual void merge(const RemoteRobotMapHypothesis& other)
  {
    RemoteKalmanPositionHypothesis::merge(other);
    RobotTypeHypothesis::merge(other);

    KalmanPositionHypothesis::mergeKalmanPosition(other.kalman);
  }

  virtual void serialize(In* in, Out* out) override
  {
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(RemoteKalmanPositionHypothesis);
    STREAM_BASE(RobotTypeHypothesis);
    STREAM_REGISTER_FINISH;
  }
};
