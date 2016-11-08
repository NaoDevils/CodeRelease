/**
 * \file MultipleBallModel.cpp
 * 
 * Implementation of class MultipleBallModel.
 * 
 * \author Heiner Walter <heiner.walter@tu-dortmund.de>
 */

#include "MultipleBallModel.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <algorithm> // std::find


// ---------- Kalman filter methods ----------

void MultipleBallModel::removeOdometry(const Pose2f& odometryOffset)
{
  // Encapsulate matrices describing the odometry offset (translation and rotation)
  // into this struct which is used to predict the kalman filter state.
  KalmanPositionTracking2D<double>::KalmanStateTransformation reverseOdometry;

  // Precompute sine and cosine of the odometry offset rotation.
  float odometryCos = std::cos(-odometryOffset.rotation);
  float odometrySin = std::sin(-odometryOffset.rotation);

  // Compute rotation and translation of the kalman state vector.
  reverseOdometry.stateRotationMatrix <<
    odometryCos, -odometrySin, 0.0, 0.0,
    odometrySin,  odometryCos, 0.0, 0.0,
    0.0, 0.0, odometryCos, -odometrySin,
    0.0, 0.0, odometrySin,  odometryCos;
  reverseOdometry.stateTranslationVector <<
    -odometryOffset.translation.x(),
    -odometryOffset.translation.y(),
    0.0,
    0.0;

  // Apply odometry offset to kalman filters of each ball hypothesis.
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    m_ballHypotheses[i].removeOdometry(reverseOdometry);
  }
}

void MultipleBallModel::motionUpdate(float timeOffset, float ballFriction)
{
  // Run prediction step of each ball hypothesis.
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    m_ballHypotheses[i].motionUpdate(timeOffset, ballFriction);
  }
}

void MultipleBallModel::sensorUpdate(
  const Vector2f& measuredBallPosition,
  float measuredDistanceToBall,
  unsigned timestamp,
  float ballPerceptValidity,
  float minDistanceForNewHypothesis,
  float initialValidityForNewHypothesis,
  const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices,
  int playernumber, const BallHypothesis::BallHypothesisTeammateInfo* teammate)
{
  sensorUpdate(
    measuredBallPosition,
    measuredDistanceToBall,
    nullptr,  // Run sensor update without measured velocity.
    timestamp,
    ballPerceptValidity,
    minDistanceForNewHypothesis,
    initialValidityForNewHypothesis,
    kalmanMatrices,
    playernumber, teammate);
}

void MultipleBallModel::sensorUpdate(
  const Vector2f& measuredBallPosition,
  float measuredDistanceToBall,
  const Vector2f* measuredBallVelocity,
  unsigned timestamp,
  float ballPerceptValidity, 
  float minDistanceForNewHypothesis,
  float initialValidityForNewHypothesis,
  const KalmanPositionTracking2D<double>::KalmanMatrices& kalmanMatrices,
  int playernumber, const BallHypothesis::BallHypothesisTeammateInfo* teammate)
{
  // Find the ball hypothesis which is nearest to the measurement.
  float distance;
  BallHypothesis* nearestHypothesis = findNearestHypothesis(measuredBallPosition, distance);
  // If new percept fitts to current best hypothesis, the validity should be increased more.
  bool updateBestHypothesis = nearestHypothesis == bestHypothesis();
  bool updateExistingHypothesis = false;

  if (nearestHypothesis != nullptr && distance < minDistanceForNewHypothesis)
  {
    // TODO: make parameter
    float measurementNoiseFactor = measuredDistanceToBall / 1000.f < 1.f ? 1.f : measuredDistanceToBall / 1000.f; // convert distance to m

    // Correct kalman filter with the perception.
    if (measuredBallVelocity == nullptr)
      nearestHypothesis->sensorUpdate(measuredBallPosition, timestamp, measurementNoiseFactor);
    else
      nearestHypothesis->sensorUpdate(measuredBallPosition, *measuredBallVelocity, timestamp, measurementNoiseFactor);

    updateExistingHypothesis = true;
  }
  else
  {
    // Add new hypothesis if all existing ones are too far.
    BallHypothesis newHypothesis(measuredBallPosition, measuredBallVelocity, initialValidityForNewHypothesis, timestamp, kalmanMatrices);
    m_ballHypotheses.push_back(newHypothesis);
    nearestHypothesis = &m_ballHypotheses.back();
    updateBestHypothesis = false;
  }

  // If a label is given, add it to the hypothesis.
  if (teammate != nullptr) nearestHypothesis->addTeammateInfo(playernumber, *teammate);
}


// ---------- Velocity methods ----------

void MultipleBallModel::initializeFrameForValidity()
{
  // Add new frame to ring buffers of all hypotheses.
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    m_ballHypotheses[i].perceptsPerSecondComputation_newFrame();
  }
}

void MultipleBallModel::updateValidity(
  unsigned int maxPerceptsPerSecond,
  float goodValidityThreshold,
  float weightOfPreviousValidity,
  float weightOfPreviousValidity_goodHypotheses)
{
  // If there is at least one real good hypothesis, reduce validity of other hypotheses faster.
  bool goodHypothesisExists = false;
  if (bestHypothesis() && bestHypothesis()->validity >= goodValidityThreshold) {
    goodHypothesisExists = true;
  }
  
  // Update validity of all hypotheses.
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    if (m_ballHypotheses[i].validity >= goodValidityThreshold)
      m_ballHypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity_goodHypotheses);
    else if (goodHypothesisExists)
      m_ballHypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity / 2);
    else
      m_ballHypotheses[i].updateValidity(maxPerceptsPerSecond, weightOfPreviousValidity);
  }
}


// ---------- Help methods ----------

BallHypothesis* MultipleBallModel::findNearestHypothesis(
  const Vector2f& measuredBallPosition, float& distance)
{
  BallHypothesis* nearestHypothesis = nullptr;
  float minDistance = -1;

  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    // Calculate distance from the perception to the current ball hypothesis.
    float currentDistance = Geometry::distance(measuredBallPosition, m_ballHypotheses[i].kalman.position());

    // Check whether the current distance is shorter than the minimum of all 
    // previous hypotheses.
    if (minDistance < 0 || currentDistance < minDistance)
    {
      nearestHypothesis = &m_ballHypotheses[i];
      minDistance = currentDistance;
    }
  }

  distance = minDistance;
  return nearestHypothesis;
}

const BallHypothesis* MultipleBallModel::bestHypothesis() const
{
  if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
  {
    return nullptr;
  }
  return &m_ballHypotheses[m_bestHypothesisIndex];
}

void MultipleBallModel::updateBestHypothesis(float minValidity, size_t minNumberOfSensorUpdates,
                                             float decreaseValidityOnChangingBestHypothesis)
{
  // If there is no hypothesis, there cannot be a best one.
  if (m_ballHypotheses.size() <= 0)
  {
    m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
    return;
  }

  // If there was no best hypothesis in the last iteration, use the 
  // current best one without limitations on validity.
  if (m_bestHypothesisIndex == static_cast<size_t>(-1)) // Maximum size_t number
  {
    minValidity = 0.f;
  }

  // Search for the best hypothesis.
  size_t bestIndex = 0;
  float bestValidity = m_ballHypotheses[bestIndex].validity;
  for (size_t i = 1; i < m_ballHypotheses.size(); i++)
  {
    // Check validity and number of sensor updates.
    if (m_ballHypotheses[i].validity > bestValidity && // highest validity and
      m_ballHypotheses[i].numberOfSensorUpdates() >= minNumberOfSensorUpdates)
    {
      // Found even better hypothesis. Remember new best one.
      bestValidity = m_ballHypotheses[i].validity;
      bestIndex = i;
    }
  }

  // Update pointer to best hypothesis if bestValidity exceeds the 
  // validity threshold.
  if (bestValidity >= minValidity)
  {
    if (m_bestHypothesisIndex < m_ballHypotheses.size() &&
      m_bestHypothesisIndex != bestIndex)
    {
      // Best hypothesis has changed.
      float lastBestValidity = m_ballHypotheses[m_bestHypothesisIndex].validity;
      if (bestValidity > lastBestValidity)
      { // For changing the best hypothesis, the validity must be truly greater than the old one.
        // Hysteresis: Reduce validity of last best hypothesis.
        m_ballHypotheses[m_bestHypothesisIndex].validity -= decreaseValidityOnChangingBestHypothesis;
        // Set new best hypothesis
        m_bestHypothesisIndex = bestIndex;
      }
    }
    else
    {
      // Set best hypothesis.
      m_bestHypothesisIndex = bestIndex;
    }
  }
}

void MultipleBallModel::cleanUpHypotheses(const FieldDimensions& theFieldDimensions, const RobotPose& theRobotPose,
  float validityThreshold, float minDistanceForSeparateHypotheses, float minAngleForSeparateHypotheses)
{
  // Do nothing if there is no ball hypothesis.
  if (m_ballHypotheses.size() <= 0) return;

  // Get validity of best hypothesis.
  float bestValidity = 1.f;
  if (m_bestHypothesisIndex < m_ballHypotheses.size())
  {
    bestValidity = m_ballHypotheses[m_bestHypothesisIndex].validity;
  }


  // ----- Remove hypotheses outside the field -----
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    // The check isInsideField requires global field coordinates. Use RobotPose
    // (from last frame) to calculate this.
    Vector2f globalPos = Transformation::robotToField(theRobotPose, m_ballHypotheses[i].kalman.position());
    if (!theFieldDimensions.isInsideCarpet(globalPos)) // Check inside carpet to allow a bit clearance for inaccuracy.
    {
      if (i != m_bestHypothesisIndex) // Don't remove best hypothesis
      {
        m_ballHypotheses.erase(m_ballHypotheses.begin() + i);
        if (m_bestHypothesisIndex == i) m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
        else if (i < m_bestHypothesisIndex && m_bestHypothesisIndex != static_cast<size_t>(-1)) m_bestHypothesisIndex--;
        i--;
      }
      else
      {
        // If best hypothesis: remove validity
        // TODO: This could be dangerous (due to inaccuracy)...
        //m_ballHypotheses[i].validity = 0.f;
      }
    }
  }


  // ----- Clean low validity hypotheses -----
  
  // Remove hypotheses with too small validity, but do not remove the best one.
  // This ensures, that at least one hypothesis is left (if set was not empty).
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    if (m_ballHypotheses[i].validity < validityThreshold && m_ballHypotheses[i].validity < bestValidity)
    {
      m_ballHypotheses.erase(m_ballHypotheses.begin() + i);
      if (m_bestHypothesisIndex == i) m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
      else if (i < m_bestHypothesisIndex && m_bestHypothesisIndex != static_cast<size_t>(-1)) m_bestHypothesisIndex--;
      i--;
    }
  }


  // ----- Clean similar hypotheses (similar to the best one) -----

  // Search for hypotheses to remove.
  for (size_t i = 0; i < m_ballHypotheses.size(); i++)
  {
    // Do nothing if current hypothesis is the best one.
    if (i != m_bestHypothesisIndex && m_bestHypothesisIndex < m_ballHypotheses.size())
    {
      // Calculate distance and angle (velocity) between best and current ball hypothesis.
      float distance = Geometry::distance(m_ballHypotheses[m_bestHypothesisIndex].kalman.position(), m_ballHypotheses[i].kalman.position());
      float angle = Geometry::angleBetween(m_ballHypotheses[m_bestHypothesisIndex].kalman.velocity(), m_ballHypotheses[i].kalman.velocity());
      bool smallVelocities = m_ballHypotheses[m_bestHypothesisIndex].kalman.velocity().norm() < minDistanceForSeparateHypotheses / 30.f &&
        m_ballHypotheses[i].kalman.velocity().norm() < minDistanceForSeparateHypotheses / 30.f;
      // Remove current hypothesis if it is too similar to the best one.
      if (distance < minDistanceForSeparateHypotheses && 
          (angle < minAngleForSeparateHypotheses || smallVelocities))
      {
        // Add all labels of hypothesis i (which will be erased/merged into the
        // best one) to the best hypothesis.
        m_ballHypotheses[m_bestHypothesisIndex].addTeammates(m_ballHypotheses[i].teammates());
        // Remove hypothesis i
        m_ballHypotheses.erase(m_ballHypotheses.begin() + i);
        if (i < m_bestHypothesisIndex && m_bestHypothesisIndex != static_cast<size_t>(-1)) m_bestHypothesisIndex--;
        i--;
      }
    }
  }
}

void MultipleBallModel::clear()
{
  m_ballHypotheses.clear();
  m_bestHypothesisIndex = static_cast<size_t>(-1); // Maximum size_t number
}

void MultipleBallModel::addHypothesis(const BallHypothesis& newHypothesis)
{
  m_ballHypotheses.push_back(newHypothesis);
}


// ---------- Debug methods ----------

void MultipleBallModel::draw() const
{
  // Draw local ball hypotheeses.

  COMPLEX_DRAWING("module:BallModelProvider:local:hypotheses")
  {
    ColorRGBA color(255, 128, 0, 255);
    ColorRGBA velocityColor(0, 0, 255, 255);

    for (const BallHypothesis& ballHypothesis : m_ballHypotheses)
    {
      // Calculate opacity depending on validity.
      unsigned char alpha = static_cast<unsigned char>(55.f + 200.f * ballHypothesis.validity);
      color.a = alpha;
      velocityColor.a = alpha;
      
      CIRCLE("module:BallModelProvider:local:hypotheses",
        ballHypothesis.kalman.positionX(),
        ballHypothesis.kalman.positionY(),
        45, 20,
        Drawings::solidPen,
        color,
        Drawings::solidBrush, // Drawings::noBrush,
        color);
      ARROW("module:BallModelProvider:local:hypotheses",
        ballHypothesis.kalman.positionX(),
        ballHypothesis.kalman.positionY(),
        ballHypothesis.kalman.positionX() + ballHypothesis.kalman.velocityX(),
        ballHypothesis.kalman.positionY() + ballHypothesis.kalman.velocityY(), 
        5, 5, color);
      DRAWTEXT("module:BallModelProvider:local:hypotheses",
        ballHypothesis.kalman.positionX() + 50,
        ballHypothesis.kalman.positionY() + 50,
        150, color, ballHypothesis.validity);

      CIRCLE("module:BallModelProvider:local:velocities",
        ballHypothesis.kalman.positionX() + (ballHypothesis.kalman.velocityX() / 30.f),
        ballHypothesis.kalman.positionY() + (ballHypothesis.kalman.velocityY() / 30.f),
        45, 10,
        Drawings::solidPen,
        velocityColor,
        Drawings::solidBrush,
        velocityColor);
      DRAWTEXT("module:BallModelProvider:local:velocities",
        ballHypothesis.kalman.positionX() + ballHypothesis.kalman.velocityX() + 70,
        ballHypothesis.kalman.positionY() + ballHypothesis.kalman.velocityY() - 100,
        150, velocityColor, ballHypothesis.kalman.velocity().norm() << "mm/s");
    }
  }
}

void RemoteMultipleBallModel::draw() const
{
  // Draw remote ball hypotheeses.

  COMPLEX_DRAWING("module:BallModelProvider:remote:hypotheses")
  {
    ColorRGBA color = ColorRGBA::black;

    for (const BallHypothesis& ballHypothesis : m_ballHypotheses)
    {
      if (ballHypothesis.validity == (bestHypothesis() == nullptr ? 0.f : bestHypothesis()->validity))
        color = ColorRGBA::black; // best hypothesis
      else
        color = ColorRGBA::gray; // all other hypotheses

      CIRCLE("module:BallModelProvider:remote:hypotheses",
        ballHypothesis.kalman.positionX(),
        ballHypothesis.kalman.positionY(),
        45, 20,
        Drawings::solidPen,
        color,
        Drawings::solidBrush, // Drawings::noBrush,
        ColorRGBA::gray);
      ARROW("module:BallModelProvider:remote:hypotheses",
        ballHypothesis.kalman.positionX(),
        ballHypothesis.kalman.positionY(),
        ballHypothesis.kalman.positionX() + ballHypothesis.kalman.velocityX(),
        ballHypothesis.kalman.positionY() + ballHypothesis.kalman.velocityY(),
        5, 5, color);
      DRAWTEXT("module:BallModelProvider:remote:hypotheses",
        ballHypothesis.kalman.positionX() + 50,
        ballHypothesis.kalman.positionY() + 50,
        150, color, ballHypothesis.validity);
      DRAWTEXT("module:BallModelProvider:remote:hypotheses",
        ballHypothesis.kalman.positionX() + 70,
        ballHypothesis.kalman.positionY() - 100,
        150, color, "p " << ballHypothesis.teammatesString());
    }
  }
}
