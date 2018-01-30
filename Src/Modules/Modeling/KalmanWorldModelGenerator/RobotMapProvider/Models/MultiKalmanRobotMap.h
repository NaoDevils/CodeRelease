/**
 * \file MultiKalmanRobotMap.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * 
 * Declaration of class \c RemoteMultiKalmanRobotMap which specializes the
 * \c RemoteMultiKalmanModel.
 * for ball modeling.
 */

#pragma once

#include "Tools/Math/Kalman/KalmanPositionTracking2D.h"
#include "Tools/PerceptsPerSecond.h"
#include "Representations/Modeling/RobotMap.h"
#include "../../Models/LocalMultiKalmanModel.h"
#include "../../Models/RemoteMultiKalmanModel.h"


/**
 * \class MultiKalmanRobotMapHypothesis
 *
 * Extension of class \c RemoteKalmanPositionHypothesis which adds functionality
 * for handling of robot map entries.
 */
class MultiKalmanRobotMapHypothesis : public RemoteKalmanPositionHypothesis
{
public:
  /**
   * Default constructor without initialization. This should not be used!
   */
  MultiKalmanRobotMapHypothesis() : RemoteKalmanPositionHypothesis() {}
  
  /**
   * \brief Constructor with initialization.
   *
   * Creates a new \c MultiKalmanRobotMapHypothesis object with a given initial
   * position and an optional initial velocity. Parameterizes the kalman filter
   * with noise matrices.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter.
   * \param [in] initialValidity The initial validity.
   * \param [in] timestamp The timestamp of the initial position measurement (in ms).
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] position The initial position (in mm).
   * \param [in] velocity Optional: The initial velocity (in mm/s)
   *                      or (0, 0) if velocity is unknown.
   */
  MultiKalmanRobotMapHypothesis(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
                                float initialValidity,
                                unsigned timestamp,
                                float perceptValidity,
                                const Vector2f& position,
                                const Vector2f& velocity = Vector2f::Zero())
  : RemoteKalmanPositionHypothesis(kalmanNoiseMatrices, initialValidity, timestamp, perceptValidity, position, velocity),
  m_robotType(RobotEstimate::RobotType::unknownRobot),
  m_lastRobotType(RobotEstimate::RobotType::unknownRobot) {}
  
  /**
   * Destructor.
   */
  ~MultiKalmanRobotMapHypothesis() {}
  
  
  // MARK: Robot type handling
  
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
  void updateRobotType(size_t minPerceptsPerSecond,
                       size_t maxPerceptsPerSecond,
                       unsigned timestamp);
  
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
  PerceptsPerSecond<1000> m_perceptsPerSecond_opponent;
  /// Stores for the last 30 frames (1000 milliseconds) how many teammate robot
  /// type sensor updates were run.
  PerceptsPerSecond<1000> m_perceptsPerSecond_teammate;
  
  /// The type of the robot map entry represented by this hypothesis. The
  /// \c robotType is generated from \c m_robotTypeValidity by the method
  /// \c updateRobotType().
  RobotEstimate::RobotType m_robotType;
  /// In this member the last valid (not unknown) robot type is saved. It will only
  /// be \c RobotType::unknownRobot in case that no robot type was detected yet.
  RobotEstimate::RobotType m_lastRobotType;
};


/**
 * Updates the given \c RobotMapEntry from a \c KalmanPositionHypothesis. The robot map
 * entry object is used in \c RobotMap::robots to represent one robot on the robot map
 * and it is filled with content extracted from the kalman hypothesis.
 * \param [in] hypothesis The \c KalmanPositionHypothesis from which the robot map entry 
 *                        is extracted.
 * \param [out] robot The \c RobotMapEntry which should be updated.
 */
void updateRobotMapEntry(const MultiKalmanRobotMapHypothesis& hypothesis, RobotMapEntry& robot);


/**
 * \class MultiKalmanRobotMap
 *
 * Specialization of \c RemoteMultiKalmanModel for robot map modeling with 
 * combined local and remote robot percepts.
 */
class MultiKalmanRobotMap : public RemoteMultiKalmanModel<MultiKalmanRobotMapHypothesis>
{
public:

  /** 
   * Default constructor creates a new and empty object.
   */
  MultiKalmanRobotMap() : RemoteMultiKalmanModel<MultiKalmanRobotMapHypothesis>() {}
  /** 
   * Destructor.
   */
  ~MultiKalmanRobotMap() {}

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best
   * to the given measurement. All other hypotheses are not changed, so their
   * validity decreases because of the motion update. If no fitting hypothesis
   * exists (see minDistanceForNewHypothesis), a new one one is created at the
   * measured position.
   * \param [in] measuredPosition The measured position.
   * \param [in] measuredDistance Distance on field between robot and
   *                              \c measuredPosition (in mm).
   * \param [in] desiredRobotType Only consider hypotheses with this robot type as
   *                              possible nearest hypotheses to update.
   * \param [in] timestamp The timestamp of the percept. This timestamp will be
   *                       saved to the hypothesis which is updated by the measurement.
   * \param [in] perceptValidity The validity of the initial position measurement in range [0,1].
   * \param [in] minDistanceForNewHypothesis If \c measuredPosition has at least this distance
   *                                         to all existing hypotheses, a new hypothesis is
   *                                         created with the percept position.
   * \param [in] initialValidityForNewHypothesis Set this validity for a new hypothesis.
   * \param [in] kalmanNoiseMatrices Fix noise matrices of the kalman filter for a new hypothesis.
   * \param [in] playerNumber The number of the teammate to add.
   * \param [in] teammate Pointer to the teammate to add.
   * \return The updated or new created hypothesis.
   */
  MultiKalmanRobotMapHypothesis* sensorUpdate(
    const Vector2f& measuredPosition,
    float measuredDistance,
    RobotEstimate::RobotType desiredRobotType,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
    int playerNumber,
    const typename MultiKalmanRobotMapHypothesis::TeammateInfo teammate);
  
  /**
   * Search for the hypothesis nearest to the percept in the set of all hypotheses
   * and return a pointer to it. If the hypotheses set is empty this method returns
   * a \c nullptr and \c distance is set to -1.
   * \param [in] measuredPosition Position of the percept (in mm).
   * \param [out] distance The distance from the percept to the nearest hypothesis.
   * \param [in] desiredRobotType Only consider hypotheses with this robot type as
   *                              possible nearest hypotheses to update.
   * \return A pointer to the nearest hypothesis.
   */
  MultiKalmanRobotMapHypothesis* findNearestHypothesis(
    const Vector2f& measuredPosition,
    float& distance,
    RobotEstimate::RobotType desiredRobotType);
  
  /**
   * Update \c m_robotType from PPS based robot type validity of all hypotheses.
   * \param [in] minPerceptsPerSecond PPS below this number is not enough to decide
   *                                  for either of the two teams.
   * \param [in] maxPerceptsPerSecond This number of PPS corresponds to a validity of 1.
   * \param [in] timestamp The current time used to update the PPS buffers.
   */
  void updateRobotTypes(size_t minPerceptsPerSecond,
                        size_t maxPerceptsPerSecond,
                        unsigned timestamp);
  
  /**
   * Draws all remote robot map hypotheses.
   */
  virtual void draw() const;
};
