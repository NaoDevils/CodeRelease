/**
 * \file BallHypothesis.cpp
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 *
 * Implementation of class BallHypothesis.
 */

#include "BallHypothesis.h"
#include "Tools/Modeling/BallPhysics.h"

#include <algorithm> // std::find (needed for compiling Nao)


void BallHypothesis::initialize(const Vector2f& position, const Vector2f* velocity)
{
  Vector2f vel;
  if (velocity == nullptr)
  {
    // Unknown initial velocity.
    vel << 0.f, 0.f;
  }
  else
    vel = *velocity;
	
  // Create initial covariance matrix.
	Eigen::Matrix4d covarianceMatrix;
	covarianceMatrix <<
		1.0, 0.0, 10.0, 0.0,
		0.0, 1.0, 0.0, 10.0,
		10.0, 0.0, 10000.0, 0.0,
		0.0, 10.0, 0.0, 10000.0;
	// Initialize kalman filter.
	kalman.initialize(position, vel, covarianceMatrix);
}

void BallHypothesis::initialize(const Vector2f& position, const Vector2f* velocity,
	const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices)
{
  Vector2f vel;
  if (velocity == nullptr)
  {
    // Unknown initial velocity.
    vel << 0.f, 0.f;
  }
  else
    vel = *velocity;
  
  // Create initial covariance matrix.
  Eigen::Matrix4d covarianceMatrix;
  covarianceMatrix <<
    1.0,   0.0,    10.0,     0.0,
    0.0,   1.0,     0.0,    10.0,
    10.0,  0.0, 10000.0,     0.0,
    0.0,  10.0,     0.0, 10000.0;
  // Initialize kalman filter.
  kalman.initialize(position, vel, covarianceMatrix, kalmanMatrices);
}

void BallHypothesis::removeOdometry(const KalmanPositionTracking2D<double>::KalmanStateTransformation& reverseOdometry)
{
  kalman.transformState(reverseOdometry);
}

void BallHypothesis::motionUpdate(float timeOffset, float ballFriction)
{
  // Calculate ball acceleration due to friction.
  // The parameter ballFriction defines the loss per second,
  // so it must be adapted to the actual time offset.
  // Use the ball motion model from class BallPhysics:
  Vector2f acceleration = BallPhysics::computeNegativeAccelerationVectorPerTimestep(kalman.velocity(), timeOffset, ballFriction);

  // Run prediction with previously computed acceleration.
  kalman.predict(acceleration, timeOffset);
}

void BallHypothesis::sensorUpdate(const Vector2f& position, unsigned timestamp, float measurementNoiseFactor)
{
  // Don't update when the timestamp is older than timeWhenLastSeen. This can
  // occur for remote hypotheses due to transmission times.
  if (timestamp < timeWhenLastSeen)
  {
    // Increase number of sensor updates.
    m_numberOfSensorUpdates++;
    return;
  }

  // Calculate time since last sensor update.
  float timeOffset = static_cast<float>(timestamp - timeWhenLastSeen) * 0.001f; // in seconds

  // Run correction.
  kalman.correct(position, timeOffset, measurementNoiseFactor);
  
  // Update timestamp of this ball hypothesis if it is newer than previous timestamp.
  if (timestamp > timeWhenLastSeen)
  {
    timeWhenLastSeen = timestamp;
  }
  // Add percept to ring buffer.
  perceptsPerSecondComputation_addPercept();
  // Increase number of sensor updates.
  m_numberOfSensorUpdates++;
}

void BallHypothesis::sensorUpdate(const Vector2f& position, const Vector2f& velocity,
                                  unsigned timestamp, float measurementNoiseFactor)
{
  // Run correction with position and velocity.
  kalman.correct(position, velocity, measurementNoiseFactor);

  // Update timestamp of this ball hypothesis if it is newer than previous timestamp.
  if (timestamp > timeWhenLastSeen)
  {
    timeWhenLastSeen = timestamp;
  }
  // Add percept to ring buffer.
  perceptsPerSecondComputation_addPercept();
  // Increase number of sensor updates.
  m_numberOfSensorUpdates++;
}

void BallHypothesis::perceptsPerSecondComputation_newFrame()
{
  // Add new frame without percept (0) to the ring buffer.
  m_perceptsPerSecond.push_front(0);
}

void BallHypothesis::perceptsPerSecondComputation_addPercept()
{
  // Add percept to the current frame in the ring buffer.
  if (m_perceptsPerSecond.empty())
  {
    // No element in ring buffer.
    // FIXME: This should not be possible, but happens!
    //        Observed in m_remoteMultipleBallModel with multiple percepts per frame.
    m_perceptsPerSecond.push_front(1);
  }
  else
  {
    m_perceptsPerSecond.set_front(m_perceptsPerSecond.front() + 1);
  }
}

unsigned int BallHypothesis::perceptsPerSecond() const
{
  return static_cast<unsigned int>(m_perceptsPerSecond.sum());
}

void BallHypothesis::updateValidity(unsigned int maxPerceptsPerSecond, float weightOfPreviousValidity)
{
  float ppsValidity = static_cast<float>(m_perceptsPerSecond.sum()) / static_cast<float>(maxPerceptsPerSecond);
  // Compute new validity as weighted mean of last validity and validity of last second (percepts/s).
  validity = (weightOfPreviousValidity * validity + ppsValidity) / (weightOfPreviousValidity + 1.f);
  // Increase validity instant if the percepts/s count is higher. => Fast increase, slow decrease.
  validity = std::max(validity, ppsValidity);
  // Cut validity to interval [0,1].
  validity = validity > 1.f ? 1.f :
             validity < 0.f ? 0.f :
             validity;
}

void BallHypothesis::updateEstimatedBallState(BallState& ballState) const
{
  // Fill given BallState object with position and velocity of the kalman filter state.
  ballState.position = kalman.position(); // (in mm)
  ballState.velocity = kalman.velocity(); // (in mm/s)
}


// ---------- Labeling methods ----------

void BallHypothesis::addTeammateInfo(int playerNumber, const BallHypothesisTeammateInfo& newTeammate)
{
  // Search for playerNumber in the current map of teammates.
  bool alreadyInMap = (m_teammates.find(playerNumber) != m_teammates.end());

  if (!alreadyInMap)
  {
    m_teammates.insert(std::pair<int, BallHypothesisTeammateInfo>(playerNumber, newTeammate));
  }
  else
  {
    BallHypothesisTeammateInfo old = m_teammates[playerNumber];
    if (newTeammate.timeWhenLastUpdated >= old.timeWhenLastUpdated)
      m_teammates[playerNumber] = newTeammate;
  }
}

void BallHypothesis::addTeammates(const std::map<int, BallHypothesis::BallHypothesisTeammateInfo>& newTeammates)
{
  for (const auto& t : newTeammates)
    addTeammateInfo(t.first, t.second);
}

unsigned BallHypothesis::timeWhenUpdatedByTeammate(int playerNumber) const
{
  // Search for playerNumber in the current map of teammates.
  bool inMap = (m_teammates.find(playerNumber) != m_teammates.end());
  
  if (!inMap) return 0; // not updated by this player
  return m_teammates.find(playerNumber)->second.timeWhenLastUpdated;
}

std::string BallHypothesis::teammatesString() const
{
  // Concatenate all labels to one string.
  std::stringstream ss;
  bool first = true;
  for (const auto& t : m_teammates)
  {
    if (!first) ss << ",";
    first = false;
    ss << t.first;
  }
  return ss.str();
}