/**
 * \file RobotTypeHypothesis.h
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 * 
 * Declaration of class \c RobotTypeHypothesis which handles robot type estimation.
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/PerceptsPerSecond.h"
#include "Representations/Modeling/RobotMap.h"


/**
 * \class RobotTypeHypothesis
 */
class RobotTypeHypothesis : public virtual Streamable
{
public:
  /**
   * Default constructor w/ default initialization.
   */
  RobotTypeHypothesis()
      : m_robotTypeValidity(0), m_perceptsPerSecond_opponent(1000), m_perceptsPerSecond_teammate(1000), m_robotType(RobotEstimate::RobotType::unknownRobot),
        m_lastRobotType(RobotEstimate::RobotType::unknownRobot){};
  /**
   * Constructor with initialization.
   *
   * \param [in] perceptDuration The duration for the PerceptPerSecond buffer.
   */
  RobotTypeHypothesis(const unsigned perceptDuration)
      : m_robotTypeValidity(0), m_perceptsPerSecond_opponent(perceptDuration), m_perceptsPerSecond_teammate(perceptDuration), m_robotType(RobotEstimate::RobotType::unknownRobot),
        m_lastRobotType(RobotEstimate::RobotType::unknownRobot){};

  /**
   * Destructor.
   */
  virtual ~RobotTypeHypothesis(){};

  /**
   * Compares the given type with the \c robotType of \c this. If one of these types
   * is \c unknownRobot they macht even if they are different.
   * \param [in] otherRobotType \c RobotType to compare.
   * IMPORTANT: Method \c updateRobotType() must be called before this method can
   *            compute a valid result.
   */
  bool matchRobotType(RobotEstimate::RobotType otherRobotType) const;

  /**
   * \param [in] timestamp The current time used to update the PPS buffers.
   */
  void addPerceptRobotType(RobotEstimate::RobotType preceptRobotType, unsigned timestamp);

  /**
   * Update \c m_robotType from PPS based robot type validity.
   * \param [in] minPerceptsPerSecond PPS below this number is not enough to decide
   *                                  for either of the two teams.
   * \param [in] maxPerceptsPerSecond This number of PPS corresponds to a validity of 1.
   * \param [in] timestamp The current time used to update the PPS buffers.
   */
  void updateRobotType(float minPerceptsPerSecond, float maxPerceptsPerSecond, unsigned timestamp);

  virtual void merge(const RobotTypeHypothesis& other);

  /**
   * Get the type of the robot map entry represented by this hypothesis. The type
   * is generated from the PPS based robot type validity by method \c updateRobotType().
   * IMPORTANT: Method \c updateRobotType() must be called before this method can
   *            return a valid type.
   */
  RobotEstimate::RobotType robotType() const { return m_robotType; }

  /**
   * Get the last valid (not unknown) robot type is saved. It will only be
   * \c RobotType::unknownRobot in case that no robot type was detected yet.
   */
  RobotEstimate::RobotType lastValidRobotType() const { return m_lastRobotType; }

  /**
   * The PPS based robot type validity in range [-1, 1] indicates the team of
   * this robot hypothesis. It is computed in each iteration by the method
   * \c updateRobotType().
   * IMPORTANT: Method \c updateRobotType() must be called before this method can
   *            return a valid type.
   * -1: teammate; 0: unknown team; 1: opponent
   */
  float robotTypeValidity() const { return m_robotTypeValidity; }
  /// Robot type validity of an opponent.
  constexpr static float ROBOT_TYPE_VALIDITY_OPPONENT = 1.f;
  /// Robot type validity of a teammate.
  constexpr static float ROBOT_TYPE_VALIDITY_TEAMMATE = -ROBOT_TYPE_VALIDITY_OPPONENT;

private:
  /// The PPS based robot type validity in range [-1, 1] indicates the team of
  /// this robot hypothesis. It is computed in each iteration by the method
  /// \c updateRobotType().
  /// -1: teammate; 0: unknown team; 1: opponent
  float m_robotTypeValidity;

  /// Stores for the last 30 frames (1000 milliseconds) how many opponent robot
  /// type sensor updates were run.
  PerceptsPerSecond m_perceptsPerSecond_opponent;
  /// Stores for the last 30 frames (1000 milliseconds) how many teammate robot
  /// type sensor updates were run.
  PerceptsPerSecond m_perceptsPerSecond_teammate;

  /// The type of the robot map entry represented by this hypothesis. The
  /// \c robotType is generated from \c m_robotTypeValidity by the method
  /// \c updateRobotType().
  RobotEstimate::RobotType m_robotType;
  /// In this member the last valid (not unknown) robot type is saved. It will only
  /// be \c RobotType::unknownRobot in case that no robot type was detected yet.
  RobotEstimate::RobotType m_lastRobotType;

public:
  /** Streaming */
  virtual void serialize(In* in, Out* out) override
  {
    STREAM_REGISTER_BEGIN;
    STREAM(m_robotTypeValidity);
    STREAM(m_robotType, RobotEstimate);
    STREAM(m_lastRobotType, RobotEstimate);
    STREAM_REGISTER_FINISH;
  }
};
