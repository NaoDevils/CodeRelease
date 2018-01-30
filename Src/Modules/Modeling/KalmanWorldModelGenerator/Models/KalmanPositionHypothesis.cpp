/**
 * \file KalmanPositionHypothesis.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of class \c KalmanPositionHypothesis.
 */

#include "KalmanPositionHypothesis.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Math/Approx.h"


//MARK: Kalman filter related methods

void KalmanPositionHypothesis::initialize(const Vector2f& position, const Vector2f& velocity)
{
  // Create initial covariance matrix.
  Eigen::Matrix4d covarianceMatrix;
  covarianceMatrix <<
    1.0, 0.0, 10.0, 0.0,
    0.0, 1.0, 0.0, 10.0,
    10.0, 0.0, 10000.0, 0.0,
    0.0, 10.0, 0.0, 10000.0;
	// Initialize kalman filter.
	kalman.initialize(position, velocity, covarianceMatrix);
}

void KalmanPositionHypothesis::initialize(const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
                                          const Vector2f& position, const Vector2f& velocity)
{
  // Create initial covariance matrix.
  Eigen::Matrix4d covarianceMatrix;
  covarianceMatrix <<
    1.0,   0.0,    10.0,     0.0,
    0.0,   1.0,     0.0,    10.0,
    10.0,  0.0, 10000.0,     0.0,
    0.0,  10.0,     0.0, 10000.0;
  // Initialize kalman filter.
  kalman.initialize(position, velocity, covarianceMatrix, kalmanNoiseMatrices);
}

void KalmanPositionHypothesis::removeOdometry(const KalmanPositionTracking2D<double>::KalmanStateTransformation& reverseOdometry)
{
  kalman.transformState(reverseOdometry);
}

void KalmanPositionHypothesis::motionUpdate(unsigned currentTimestamp, float friction)
{
  // Calculate time since last motionUpdate.
  float timeOffset = static_cast<float>(currentTimestamp - m_lastMotionUpdateTimeStamp) * 0.001f; // in seconds
  // Save current timestamp for next iteration.
  m_lastMotionUpdateTimeStamp = currentTimestamp;

  // Calculate acceleration due to friction.
  // The parameter friction defines the loss per second,
  // so it must be adapted to the actual time offset.
  // Use the ball motion model from class BallPhysics:
  Vector2f acceleration = BallPhysics::computeNegativeAccelerationVectorPerTimestep(kalman.velocity(), timeOffset, friction);

  // Run prediction with previously computed acceleration.
  kalman.predict(acceleration, timeOffset);

  // Update pps tracker.
  m_perceptsPerSecond.updateCurrentTime(currentTimestamp);
}

void KalmanPositionHypothesis::sensorUpdate(const Vector2f& position,
                                            unsigned timestamp, float perceptValidity,
                                            float measurementNoiseFactor)
{
  // Increase number of sensor updates (even if the percept is older than timeWhenLastSeen).
  m_numberOfSensorUpdates++;
  // Don't update when the timestamp is older than timeWhenLastSeen. This can
  // occur for remote hypotheses due to transmission times.
  if (timestamp < timeWhenLastSeen) return;

  // Calculate time since last sensor update.
  float timeOffset = static_cast<float>(timestamp - timeWhenLastSeen) * 0.001f; // in seconds

  // Run correction.
  kalman.correct(position, timeOffset, measurementNoiseFactor);
  
  // Update timestamp of this hypothesis.
  timeWhenLastSeen = timestamp;
  // Add percept to the pps tracker.
  m_perceptsPerSecond.addPercept(timestamp, perceptValidity);
  // Save unfiltered percept.
  lastPerception = position;
}

void KalmanPositionHypothesis::sensorUpdate(const Vector2f& position, const Vector2f& velocity,
                                            unsigned timestamp, float perceptValidity,
                                            float measurementNoiseFactor)
{
  // Increase number of sensor updates (even if the percept is older than timeWhenLastSeen).
  m_numberOfSensorUpdates++;
  // Don't update when the timestamp is older than timeWhenLastSeen. This can
  // occur for remote hypotheses due to transmission times.
  if (timestamp < timeWhenLastSeen) return;

  // Run correction with position and velocity.
  kalman.correct(position, velocity, measurementNoiseFactor);

  // Update timestamp of this hypothesis.
  timeWhenLastSeen = timestamp;
  // Add percept to the pps tracker.
  m_perceptsPerSecond.addPercept(timestamp, perceptValidity);
  // Save unfiltered percept.
  lastPerception = position;
}


//MARK: Validity related methods

size_t KalmanPositionHypothesis::perceptsPerSecond() const
{
  return m_perceptsPerSecond.pps();
}

size_t KalmanPositionHypothesis::perceptsPerSecond(unsigned currentTimestamp)
{
  return m_perceptsPerSecond.pps(currentTimestamp);
}

float KalmanPositionHypothesis::meanPerceptValidity() const
{
  return m_perceptsPerSecond.meanPerceptValidity();
}

float KalmanPositionHypothesis::meanPerceptValidity(unsigned currentTimestamp)
{
  return m_perceptsPerSecond.meanPerceptValidity(currentTimestamp);
}

void KalmanPositionHypothesis::updateValidity(size_t maxPerceptsPerSecond, float weightOfPreviousValidity)
{
  // Use PPS multiplied with the mean percept validity tracked over the same time as the PPS (value smaller than pure PPS).
  // This gives an influence of the percept validity on the resulting model validity.
  float pps = m_perceptsPerSecond.meanPerceptValidity() * static_cast<float>(m_perceptsPerSecond.pps());
  float ppsValidity = pps / static_cast<float>(maxPerceptsPerSecond);
  // Compute new validity as weighted mean of last validity and validity of last second (percepts/s).
  validity = (weightOfPreviousValidity * validity + 1.f * ppsValidity) / (weightOfPreviousValidity + 1.f);
  // Increase validity instant if the percepts/s count is higher. => Fast increase, slow decrease.
  validity = std::max(validity, ppsValidity);
  // Cut validity to interval [0,1].
  validity = validity > 1.f ? 1.f :
             validity < 0.f ? 0.f :
             validity;
}

void KalmanPositionHypothesis::increaseFilterCovariance(double factor)
{
  if (!Approx::isEqual(factor, 1.0))
  {
    // Scale whole 4x4 covariance matrix.
    kalman.covarianceMatrix *= factor;
  }
}

void KalmanPositionHypothesis::increaseFilterCovariance(double positionFactor, double velocityFactor)
{
  if (!Approx::isEqual(positionFactor, 1.0))
  {
    // Scale 2x2 position part of covariance matrix.
    kalman.covarianceMatrix.block<2, 2>(0, 0) *= positionFactor;
  }
  if (!Approx::isEqual(velocityFactor, 1.0))
  {
    // Scale 2x2 velocity part of covariance matrix.
    kalman.covarianceMatrix.block<2, 2>(2, 2) *= velocityFactor;
  }
}


//MARK: Helper methods

void KalmanPositionHypothesis::merge(const KalmanPositionHypothesis& source)
{
  if (source.timeWhenLastSeen > timeWhenLastSeen)
  {
    // source has a more recent percept than *this.
    // Use source.lastPerception for an sensor Update of *this.
    sensorUpdate(source.lastPerception, source.timeWhenLastSeen, source.validity);
    m_numberOfSensorUpdates--; // This sensor update is later added with source.m_numberOfSensorUpdates.
  }
  // Add perception history.
  m_numberOfSensorUpdates += source.m_numberOfSensorUpdates;
  m_perceptsPerSecond.addPercepts(source.m_perceptsPerSecond);
}
