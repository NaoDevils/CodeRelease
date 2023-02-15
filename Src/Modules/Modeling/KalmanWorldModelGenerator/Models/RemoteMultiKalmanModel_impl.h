/**
 * \file RemoteMultiKalmanModel_impl.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of class \c RemoteMultiKalmanModel.
 */

#include "RemoteMultiKalmanModel.h"


//MARK: Kalman filter related methods

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* RemoteMultiKalmanModel<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
    int playerNumber,
    const typename hypothesis_t::TeammateInfo teammate)
{
  return sensorUpdate(measuredPosition,
      measuredDistance,
      nullptr, // Run sensor update without measured velocity.
      timestamp,
      perceptValidity,
      minDistanceForNewHypothesis,
      initialValidityForNewHypothesis,
      kalmanNoiseMatrices,
      playerNumber,
      teammate);
}

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* RemoteMultiKalmanModel<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    const Vector2f* measuredVelocity,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
    int playerNumber,
    const typename hypothesis_t::TeammateInfo teammate)
{
  RemoteKalmanPositionHypothesis* hypothesis = MultiKalmanModel<hypothesis_t>::sensorUpdate(
      measuredPosition, measuredDistance, measuredVelocity, timestamp, perceptValidity, minDistanceForNewHypothesis, initialValidityForNewHypothesis, kalmanNoiseMatrices);

  // Add a teammate label to the hypothesis.
  hypothesis->addTeammateInfo(playerNumber, teammate);
  return hypothesis;
}
