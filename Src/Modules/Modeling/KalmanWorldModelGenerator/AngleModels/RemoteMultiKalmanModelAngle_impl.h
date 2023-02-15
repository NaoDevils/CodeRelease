/**
 * \file RemoteMultiKalmanModelAngle_impl.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * \author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 *
 * Implementation of class \c RemoteMultiKalmanModelAngle.
 */

#include "RemoteMultiKalmanModelAngle.h"


//MARK: Kalman filter related methods

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* RemoteMultiKalmanModelAngle<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    unsigned timestamp,
    float perceptValidity,
    const Vector2a& mergeAngleDiff,
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
      mergeAngleDiff,
      initialValidityForNewHypothesis,
      kalmanNoiseMatrices,
      playerNumber,
      teammate);
}

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* RemoteMultiKalmanModelAngle<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    const Vector2f* measuredVelocity,
    unsigned timestamp,
    float perceptValidity,
    const Vector2a& mergeAngleDiff,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
    int playerNumber,
    const typename hypothesis_t::TeammateInfo teammate)
{
  RemoteKalmanPositionHypothesis* hypothesis = MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::sensorUpdate(
      measuredPosition, measuredDistance, measuredVelocity, timestamp, perceptValidity, mergeAngleDiff, initialValidityForNewHypothesis, kalmanNoiseMatrices);

  // Add a teammate label to the hypothesis.
  hypothesis->addTeammateInfo(playerNumber, teammate);
  return hypothesis;
}
