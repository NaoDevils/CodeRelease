/**
 * \file RemoteMultiKalmanModelAngle.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * \author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 *
 * Declaration of class RemoteMultiKalmanModelAngle.
 * This class is an extension to \c MultiKalmanModelAngle
 */

#pragma once

#include "GlobalMultiKalmanModelAngle.h"
#include "Modules/Modeling/KalmanWorldModelGenerator/Models/RemoteKalmanPositionHypothesis.h"


/**
  * \class RemoteMultiKalmanModelAngle
  *
  * This class is an extension to \c MultiKalmanModelAngle for handling of remote
  * models received from teammates. It adds different sensor update methods
  * which should be used for remote models instead of the sensor update methods
  * from \c MultiKalmanModelAngle.
  * Unlike local models the remote model stores positions and velocities in
  * absolute field coordinates.
  * \see MultiKalmanModelAngle, RemoteKalmanPositionHypothesis
  */
//template <typename hypothesis_t = RemoteKalmanPositionHypothesis, unsigned towardsNoOfModels = 4>
template <typename hypothesis_t, bool towardsOneModel> class RemoteMultiKalmanModelAngle : public GlobalMultiKalmanModelAngle<hypothesis_t, towardsOneModel>
{
  // Check at compile-time that hypothesis_t is derived from class RemoteKalmanPositionHypothesis.
  static_assert(std::is_base_of<RemoteKalmanPositionHypothesis, hypothesis_t>::value, "hypothesis_t not derived from class RemoteKalmanPositionHypothesis");

public:
  /**
   * Default constructor creates a new and empty \c RemoteMultiKalmanModelAngle object.
   */
  RemoteMultiKalmanModelAngle() : GlobalMultiKalmanModelAngle<hypothesis_t, towardsOneModel>() {}
  /**
   * Constructor setting the perceptDuration.
   * Instead of calling this constructor a deriving class can also override the getPerceptDuration method.
   *
   * \param [in] perceptDuration, The duration (in ms) percepts get buffered for identifying the validity of a hypothesis.
   */
  RemoteMultiKalmanModelAngle(unsigned perceptDuration) : GlobalMultiKalmanModelAngle<hypothesis_t, towardsOneModel>(perceptDuration) {}
  /**
   * Destructor.
   */
  ~RemoteMultiKalmanModelAngle() {}


  //MARK: Kalman model related methods

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
  hypothesis_t* sensorUpdate(const Vector2f& measuredPosition,
      float measuredDistance,
      unsigned timestamp,
      float perceptValidity,
      const Vector2a& mergeAngleDiff,
      float initialValidityForNewHypothesis,
      const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      int playerNumber,
      const typename hypothesis_t::TeammateInfo teammate);

  /**
   * \brief Correction
   *
   * This method performs the correction step of the hypothesis which fits best
   * to the given measurement. All other hypotheses are not changed, so their
   * validity decreases because of the motion update. If no fitting hypothesis
   * exists (see minDistanceForNewHypothesis), a new one one is created at the
   * measured position.
   * \param [in] measuredPosition The measured position.
   * \param [in] measuredVelocity Optional: A pointer to the measurement of the
   *                              velocity or \c nullptr if the velocity is not
   *                              measured.
   * \param [in] measuredDistance Distance on field between robot and
   *                              \c measuredPosition (in mm).
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
  hypothesis_t* sensorUpdate(const Vector2f& measuredPosition,
      float measuredDistance,
      const Vector2f* measuredVelocity,
      unsigned timestamp,
      float perceptValidity,
      const Vector2a& mergeAngleDiff,
      float initialValidityForNewHypothesis,
      const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
      int playerNumber,
      const typename hypothesis_t::TeammateInfo teammate);
};

#include "RemoteMultiKalmanModelAngle_impl.h"