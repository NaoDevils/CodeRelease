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
#include "RemoteRobotMapHypothesis.h"


/**
 * \class MultiKalmanRobotMap
 *
 * Specialization of \c RemoteMultiKalmanModel for robot map modeling with 
 * combined local and remote robot percepts.
 */
class MultiKalmanRobotMap : public RemoteMultiKalmanModel<RemoteRobotMapHypothesis, false>
{
public:
  /** 
   * Default constructor creates a new and empty object.
   */
  MultiKalmanRobotMap() : RemoteMultiKalmanModel<RemoteRobotMapHypothesis, false>() {}
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
  RemoteRobotMapHypothesis* sensorUpdate(const Vector2f& measuredPosition,
      float measuredDistance,
      RobotEstimate::RobotType desiredRobotType,
      unsigned timestamp,
      float perceptValidity,
      float minDistanceForNewHypothesis,
      float initialValidityForNewHypothesis,
      const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      int playerNumber,
      const typename RemoteRobotMapHypothesis::TeammateInfo teammate);

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
  RemoteRobotMapHypothesis* findNearestHypothesis(const Vector2f& measuredPosition, float& distance, RobotEstimate::RobotType desiredRobotType);

  /**
   * Update \c m_robotType from PPS based robot type validity of all hypotheses.
   * \param [in] minPerceptsPerSecond PPS below this number is not enough to decide
   *                                  for either of the two teams.
   * \param [in] maxPerceptsPerSecond This number of PPS corresponds to a validity of 1.
   * \param [in] timestamp The current time used to update the PPS buffers.
   */
  void updateRobotTypes(float minPerceptsPerSecond, float maxPerceptsPerSecond, unsigned timestamp);

  /**
   * Draws all remote robot map hypotheses.
   */
  virtual void draw() const;
};
