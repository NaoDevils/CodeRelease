/**
 * \file MultiKalmanModelAngle_impl.h
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 * \author <a href="mailto:arne.moos@tu-dortmund.de">Arne Moos</a>
 *
 * Implementation of class \c MultiKalmanModelAngle.
 */

#include "MultiKalmanModelAngle.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Math/Transformation.h"

#include <algorithm> // std::find

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    unsigned timestamp,
    float perceptValidity,
    const Vector2a& mergeAngleDiff,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices)
{
  return sensorUpdate(measuredPosition,
      measuredDistance,
      nullptr, // Run sensor update without measured velocity.
      timestamp,
      perceptValidity,
      mergeAngleDiff,
      initialValidityForNewHypothesis,
      kalmanNoiseMatrices);
}

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    const Vector2f* measuredVelocity,
    unsigned timestamp,
    float perceptValidity,
    const Vector2a& mergeAngleDiff,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices)
{
  // Reset index of best hypothesis. This must be recalculated each frame.
  resetBestHypothesisIndex();

  // Find the hypothesis which is nearest to the measurement.
  Vector2a angleDiff;
  float distance;
  hypothesis_t* nearestHypothesis = findNearestHypothesis(angleDiff, distance, measuredPosition);

  // TODO: make parameter
  float measurementNoiseFactor = powf(1.25, measuredDistance / 1000.f < 1.f ? 1.f : measuredDistance / 1000.f); // convert distance to m
  float angleDistanceFactor = 9.f / (measuredDistance / 1000.f);

  Vector2a localMergeAngleDiff(mergeAngleDiff.x() * angleDistanceFactor, mergeAngleDiff.y() * angleDistanceFactor);

  if (nearestHypothesis != nullptr)
  {
    localMergeAngleDiff.x() = localMergeAngleDiff.x() * nearestHypothesis->mergeFactor;
    localMergeAngleDiff.y() = localMergeAngleDiff.y() * nearestHypothesis->mergeFactor;
  }

  // Merge hypotheses
  if (nearestHypothesis != nullptr && (distance < 300.f || (angleDiff.y() < localMergeAngleDiff.y() && angleDiff.x() < localMergeAngleDiff.x())))
  {
    // Set measurementNoiseMatrix according to the direction from robot to measurement position.
    // In this direction (^= distance) the noise is set to a higher value than in the orthogonal direction (^= angle).
    float max = static_cast<float>(nearestHypothesis->kalman.matrices.noise.maxMeasurementNoise);
    max *= measurementNoiseFactor;
    nearestHypothesis->kalman.matrices.noise.measurementNoiseMatrix = Covariance::create((Vector2f() << max, max / 5.f).finished(), measuredPosition.angle()).cast<double>();

    // Correct kalman filter with the perception.
    if (measuredVelocity == nullptr)
      nearestHypothesis->sensorUpdate(measuredPosition, timestamp, perceptValidity);
    else
      nearestHypothesis->sensorUpdate(measuredPosition, *measuredVelocity, timestamp, perceptValidity);
  }
  // Create new hypothesis
  else
  {
    // Add new hypothesis if all existing ones are too far.
    m_hypotheses.push_back(hypothesis_t(
        kalmanNoiseMatrices, initialValidityForNewHypothesis, timestamp, perceptValidity, measuredPosition, getPerceptDuration(), measuredVelocity != nullptr ? *measuredVelocity : Vector2f::Zero()));
    nearestHypothesis = &m_hypotheses.back();
  }

  return nearestHypothesis;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::resetBestHypothesisIndex()
{
  // Reset index of best hypothesis. This must be recalculated each frame.
  m_lastBestHypothesisIndex = m_bestHypothesisIndex;
  m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
}

template <typename hypothesis_t, bool towardsOneModel>
hypothesis_t* MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::findNearestHypothesis(Vector2a& angleDiff, float& distance, const Vector2f& measuredPosition)
{
  hypothesis_t* nearestHypothesisAngle = nullptr;
  Angle minAngleDiff = -1;
  Angle minAngleDiffX = -1;
  Angle minAngleDiffY = -1;
  float minDistance = -1;

  Vector2a measuredPositionRelativeAngles, hypothesisPositionRelativeAngles;
  getAngles(measuredPositionRelativeAngles, measuredPosition);

  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    hypothesis_t& actualHypothesis = m_hypotheses[i];
    Vector2f position = actualHypothesis.kalman.position();
    position = BallPhysics::propagateBallPosition(position, actualHypothesis.kalman.velocity(), 30.f / 1000.f, friction);

    getAngles(hypothesisPositionRelativeAngles, position);
    float currentDistance = Geometry::distance(measuredPosition, position);

    // Calculate head angles between the perception and the current hypothesis.
    Angle yDiff = std::abs(measuredPositionRelativeAngles.y() - hypothesisPositionRelativeAngles.y());
    Angle xDiff = std::abs(measuredPositionRelativeAngles.x() - hypothesisPositionRelativeAngles.x());

    // Check whether the angle diff sum is smaller than the minimum of all previous hypotheses.
    if (minAngleDiff < 0 || (xDiff + yDiff) < minAngleDiff)
    {
      nearestHypothesisAngle = &actualHypothesis;
      minAngleDiff = (xDiff + yDiff);
      minAngleDiffY = yDiff;
      minAngleDiffX = xDiff;
      minDistance = currentDistance;
    }
  }

  angleDiff.y() = minAngleDiffY;
  angleDiff.x() = minAngleDiffX;
  distance = minDistance;

  return nearestHypothesisAngle;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::getAngles(Vector2a& angles, const Vector2f& relativePosition)
{
  angles.x() = std::atan2(relativePosition.y(), relativePosition.x());
  angles.y() = std::atan2(cameraMatrix.translation.z(), relativePosition.norm());
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::removeOdometry(const Pose2f& odometryOffset)
{
  // Encapsulate matrices describing the odometry offset (translation and rotation)
  // into this struct which is used to predict the kalman filter state.
  KalmanPositionTracking2D<double>::KalmanStateTransformation reverseOdometry;

  // Precompute sine and cosine of the odometry offset rotation.
  float odometryCos = std::cos(-odometryOffset.rotation);
  float odometrySin = std::sin(-odometryOffset.rotation);

  // Compute rotation and translation of the kalman state vector.
  reverseOdometry.stateRotationMatrix << odometryCos, -odometrySin, 0.0, 0.0, odometrySin, odometryCos, 0.0, 0.0, 0.0, 0.0, odometryCos, -odometrySin, 0.0, 0.0, odometrySin, odometryCos;

  reverseOdometry.stateTranslationVector << -odometryOffset.translation.x(), -odometryOffset.translation.y(), 0.0, 0.0;

  // Apply odometry offset to kalman filters of each hypothesis.
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    m_hypotheses[i].removeOdometry(reverseOdometry);
  }
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::motionUpdate(unsigned currentTimestamp)
{
  // Run prediction step of each hypothesis.
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    m_hypotheses[i].motionUpdate(currentTimestamp, friction);
  }
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::updateValidity(float maxPerceptsPerSecond, float goodValidityThreshold, float weightOfPreviousValidity, float weightOfPreviousValidity_goodHypotheses)
{
  // If there is at least one real good hypothesis, reduce validity of other hypotheses faster.
  bool goodHypothesisExists = false;
  if constexpr (towardsOneModel)
  {
    if (bestHypothesis() && bestHypothesis()->validity >= goodValidityThreshold)
    {
      goodHypothesisExists = true;
    }
  }

  // Update validity of all hypotheses.
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    if (m_hypotheses[i].validity >= goodValidityThreshold)
      m_hypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity_goodHypotheses);
    else if (goodHypothesisExists)
      m_hypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity / 2);
    else
      m_hypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity);
  }
}

template <typename hypothesis_t, bool towardsOneModel> const hypothesis_t* MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::bestHypothesis()
{
  updateBestHypothesisIndexIfNecessary();
  if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
  {
    return nullptr;
  }
  return &m_hypotheses[m_bestHypothesisIndex];
}

template <typename hypothesis_t, bool towardsOneModel> std::size_t MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::updateBestHypothesisIndexIfNecessary()
{
  if (m_bestHypothesisIndex == static_cast<size_t>(-1) && m_hypotheses.size() > 0)
  {
    // Compute index of best hypothesis.
    updateBestHypothesis();
  }
  return m_bestHypothesisIndex;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::removeHypothesis(size_t index)
{
  m_hypotheses.erase(m_hypotheses.begin() + index);
  if (m_bestHypothesisIndex == index)
    m_bestHypothesisIndex = std::numeric_limits<size_t>::max();
  else if (index < m_bestHypothesisIndex && m_bestHypothesisIndex != std::numeric_limits<size_t>::max())
    m_bestHypothesisIndex--;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::updateBestHypothesis()
{
  // If there is no hypothesis, there cannot be a best one.
  if (m_hypotheses.size() <= 0)
  {
    m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
    return;
  }

  float minValidity = minValidityForChangingBestHypothesis;

  // If there was no best hypothesis in the last iteration or there is only one hypothesis,
  // use the current best one without limitations on validity.
  if (m_lastBestHypothesisIndex == static_cast<size_t>(-1) || m_hypotheses.size() == size_t(1))
  {
    minValidity = 0.f;
  }

  // Search for the best hypothesis.
  size_t bestIndex = std::numeric_limits<size_t>::max();
  float bestValidity = -1.f;
  float bestDistance = std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    float dist = m_hypotheses[i].kalman.position().norm();
    // Check validity and number of sensor updates.
    if ((m_hypotheses[i].validity > bestValidity || (m_hypotheses[i].validity == bestValidity && dist < bestDistance)) // highest validity or equal validity but nearer
        && (m_hypotheses[i].numberOfSensorUpdates() >= minNumberOfSensorUpdatesForBestHypothesis || bestIndex == static_cast<size_t>(-1))) // and has updates or is the only one so far
    {
      // Found even better hypothesis. Remember new best one.
      bestIndex = i;
      bestValidity = m_hypotheses[bestIndex].validity;
      bestDistance = dist;
    }
  }

  // Update pointer to best hypothesis if bestValidity exceeds the
  // validity threshold.
  if (bestValidity >= minValidity)
  {
    if (m_lastBestHypothesisIndex < m_hypotheses.size() && m_lastBestHypothesisIndex != bestIndex)
    {
      // Best hypothesis has changed.
      float lastBestValidity = m_hypotheses[m_lastBestHypothesisIndex].validity;
      if (bestValidity > lastBestValidity)
      { // For changing the best hypothesis, the validity must be truly greater than the old one.
        // Hysteresis: Reduce validity of last best hypothesis.
        m_hypotheses[m_lastBestHypothesisIndex].validity -= decreaseValidityOnChangingBestHypothesis;
      }
    }
  }

  // Set best hypothesis.
  m_bestHypothesisIndex = bestIndex;
}

//MARK: Kalman filter related methods

//MARK: Validity methods

//MARK: Helper methods

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::setParametersForUpdateBestHypothesis(float minValidity, std::size_t minNumber, float decreaseValidity)
{
  minValidityForChangingBestHypothesis = minValidity;
  minNumberOfSensorUpdatesForBestHypothesis = minNumber;
  decreaseValidityOnChangingBestHypothesis = decreaseValidity;
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::increaseUncertainty(double positionFactor, double velocityFactor, bool onlyBestHypothesis)
{
  updateBestHypothesisIndexIfNecessary();
  if (onlyBestHypothesis)
  {
    if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
      return; // No best hypothesis available
    // Increase velocity covariance. Position covariance stays unchanged.
    m_hypotheses[m_bestHypothesisIndex].increaseFilterCovariance(positionFactor, velocityFactor);
  }
  else
  {
    for (size_t i = 0; i < m_hypotheses.size(); i++)
    {
      // Increase velocity covariance. Position covariance stays unchanged.
      m_hypotheses[i].increaseFilterCovariance(positionFactor, velocityFactor);
    }
  }
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::updateMaxMeasurementNoise(double maxMeasurementNoise, bool onlyBestHypothesis)
{
  updateBestHypothesisIndexIfNecessary();
  if (onlyBestHypothesis)
  {
    if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
      return; // No best hypothesis available
    m_hypotheses[m_bestHypothesisIndex].updateMaxMeasurementNoise(maxMeasurementNoise);
  }
  else
  {
    for (size_t i = 0; i < m_hypotheses.size(); i++)
    {
      // Increase velocity covariance. Position covariance stays unchanged.
      m_hypotheses[i].updateMaxMeasurementNoise(maxMeasurementNoise);
    }
  }
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::setRobotPoseCameraMatrixAndFriction(const RobotPose& robotPose, const CameraMatrix& cameraMatrix, float friction)
{
  this->robotPose = robotPose;
  this->cameraMatrix = cameraMatrix;
  this->friction = friction;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::hypothesisKicked(bool longkick)
{
  updateBestHypothesisIndexIfNecessary();
  if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
    return; // No best hypothesis available

  // Increase velocity covariance. Position covariance stays unchanged.
  //m_hypotheses[m_bestHypothesisIndex].kalman.state(2) += 20.f;
  //m_hypotheses[m_bestHypothesisIndex].kalman.state(3) += 5.f;

  Vector2f position = m_hypotheses[m_bestHypothesisIndex].kalman.position();
  Vector2f velocity = m_hypotheses[m_bestHypothesisIndex].kalman.velocity();
  float addingVelocity = 200.f;
  float mergeFactor = 3.f;
  if (longkick)
  {
    addingVelocity = 400.f;
  }

  Vector2f relativeVelocity = Transformation::fieldVelocityToRobot(robotPose, velocity);
  relativeVelocity.x() += addingVelocity;
  velocity = Transformation::robotToFieldVelocity(robotPose, relativeVelocity);

  m_hypotheses[m_bestHypothesisIndex].kalman.state(2) = velocity.x();
  m_hypotheses[m_bestHypothesisIndex].kalman.state(3) = velocity.y();
  m_hypotheses[m_bestHypothesisIndex].mergeFactor = mergeFactor;
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::stopBestHypothesis()
{
  updateBestHypothesisIndexIfNecessary();
  if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
    return; // No best hypothesis available

  hypothesis_t& actualHypothesis = m_hypotheses[m_bestHypothesisIndex];
  actualHypothesis.kalman.state(2) = 0.f;
  actualHypothesis.kalman.state(3) = 0.f;
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::cleanUpHypothesesOutsideField(const FieldDimensions& theFieldDimensions, const RobotPose& theRobotPose, float fieldBorderThreshold)
{
  updateBestHypothesisIndexIfNecessary();
  // ----- Remove hypotheses outside the field -----

  ASSERT(fieldBorderThreshold > 0.f);
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    // The check isInsideField requires global field coordinates. Use RobotPose
    // (from last frame) to calculate this.
    Vector2f globalPos = m_hypotheses[i].kalman.position();
    if (usesRelativeCoordinates())
      globalPos = Transformation::robotToField(theRobotPose, globalPos);
    // Hypothesis must be outside the field border ...
    if (!theFieldDimensions.fieldBorder.isInside(globalPos))
    {
      // ... and outside the field border + threshold to be removed.
      // Get closest distance from position to field border.
      Vector2f closestPosOnBorder = theFieldDimensions.fieldBorder.getClosestPoint(globalPos);
      float distance = (closestPosOnBorder - globalPos).norm();
      // Check if hypothesis is inside field border + threshold to allow a bit clearance for inaccuracy.
      if (distance > fieldBorderThreshold)
      {
        if constexpr (!towardsOneModel)
        {
          removeHypothesis(i);
          i--;
        }
        // Do nothing if current hypothesis is the best one.
        else if (i != m_bestHypothesisIndex && m_bestHypothesisIndex < m_hypotheses.size() && m_hypotheses.size() > 1)
        {
          removeHypothesis(i);
          i--;
        }
        else
        {
          Vector2f closestPosOnBorderRel = Transformation::fieldToRobot(theRobotPose, closestPosOnBorder);
          m_hypotheses[i].kalman.state(0) = closestPosOnBorderRel.x();
          m_hypotheses[i].kalman.state(1) = closestPosOnBorderRel.y();
          m_hypotheses[i].kalman.state(2) = 0.f;
          m_hypotheses[i].kalman.state(3) = 0.f;
        }
      }
    }
  }
}

template <typename hypothesis_t, bool towardsOneModel>
void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::cleanUpHypothesesLowValidity(float validityThreshold, bool saveBestHypothesis)
{
  // Do nothing if there is no hypothesis.
  if (m_hypotheses.size() <= 0)
    return;

  // Get validity of best hypothesis.
  float bestValidity = 1.f;
  if (saveBestHypothesis)
  {
    updateBestHypothesisIndexIfNecessary();
    if (m_bestHypothesisIndex == static_cast<size_t>(-1) && m_lastBestHypothesisIndex < m_hypotheses.size())
    {
      m_bestHypothesisIndex = m_lastBestHypothesisIndex;
    }
    if (m_bestHypothesisIndex < m_hypotheses.size())
    {
      bestValidity = m_hypotheses[m_bestHypothesisIndex].validity;
    }
  }

  if (bestValidity < validityThreshold)
    bestValidity = validityThreshold;

  // ----- Clean low validity hypotheses -----

  // Remove hypotheses with too small validity, but do not remove the best one.
  // This ensures, that at least one hypothesis is left (if set was not empty).
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    if (m_hypotheses[i].validity < validityThreshold && m_hypotheses[i].validity < bestValidity && m_hypotheses.size() > 1)
    {
      if constexpr (!towardsOneModel)
      {
        removeHypothesis(i);
        i--;
      }
      // Do nothing if current hypothesis is the best one.
      else if (i != m_bestHypothesisIndex && m_bestHypothesisIndex < m_hypotheses.size() && m_hypotheses.size() > 1)
      {
        removeHypothesis(i);
        i--;
      }
    }
  }
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::cleanUpHypothesesSimilarToBestOne(const Vector2a& mergeAngleDiff)
{
  updateBestHypothesisIndexIfNecessary();
  if (m_bestHypothesisIndex == static_cast<size_t>(-1))
    return;

  // ----- Clean similar hypotheses (similar to the best one) -----

  Vector2a bestHypothesisPositionRelativeAngles, hypothesisPositionRelativeAngles;
  hypothesis_t& bestHypothesis = m_hypotheses[m_bestHypothesisIndex];
  getAngles(bestHypothesisPositionRelativeAngles, bestHypothesis.kalman.position());

  // Search for hypotheses to remove.
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    // Do nothing if current hypothesis is the best one.
    if (i != m_bestHypothesisIndex && m_bestHypothesisIndex < m_hypotheses.size())
    {
      hypothesis_t& actualHypothesis = m_hypotheses[i];
      getAngles(hypothesisPositionRelativeAngles, actualHypothesis.kalman.position());
      // Calculate head angles between the perception and the current hypothesis.
      Angle yDiff = std::abs(bestHypothesisPositionRelativeAngles.y() - hypothesisPositionRelativeAngles.y());
      Angle xDiff = std::abs(bestHypothesisPositionRelativeAngles.x() - hypothesisPositionRelativeAngles.x());

      // Remove current hypothesis if it is too similar to the best one.
      if (yDiff < mergeAngleDiff.y() && xDiff < mergeAngleDiff.x())
      {
        m_hypotheses[m_bestHypothesisIndex].merge(actualHypothesis);
        removeHypothesis(i);
        i--;
      }

      //// Calculate distance and angle (velocity) between best and current hypothesis.
      //float distance = Geometry::distance(m_hypotheses[m_bestHypothesisIndex].kalman.position(), m_hypotheses[i].kalman.position());
      //float angle = Geometry::angleBetween(m_hypotheses[m_bestHypothesisIndex].kalman.velocity(), m_hypotheses[i].kalman.velocity());
      //bool smallVelocities = m_hypotheses[m_bestHypothesisIndex].kalman.velocity().norm() < minDistanceForSeparateHypotheses / 30.f &&
      //  m_hypotheses[i].kalman.velocity().norm() < minDistanceForSeparateHypotheses / 30.f;
      //// Remove current hypothesis if it is too similar to the best one.
      //if (distance < minDistanceForSeparateHypotheses &&
      //    (angle < minAngleForSeparateHypotheses || smallVelocities))
      //{
      //  m_hypotheses[m_bestHypothesisIndex].merge(m_hypotheses[i]);
      //  // Remove hypothesis i
      //  m_hypotheses.erase(m_hypotheses.begin() + i);
      //  if (i < m_bestHypothesisIndex && m_bestHypothesisIndex != static_cast<size_t>(-1)) m_bestHypothesisIndex--;
      //  i--;
      //}
    }
  }
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::clear()
{
  m_hypotheses.clear();
  m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::addHypothesis(const hypothesis_t& newHypothesis)
{
  m_hypotheses.push_back(newHypothesis);
}

template <typename hypothesis_t, bool towardsOneModel> void MultiKalmanModelAngle<hypothesis_t, towardsOneModel>::addHypothesis(hypothesis_t&& newHypothesis)
{
  m_hypotheses.push_back(std::forward<hypothesis_t>(newHypothesis));
}
