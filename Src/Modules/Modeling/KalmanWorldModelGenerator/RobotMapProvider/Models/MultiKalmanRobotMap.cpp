/**
 * \file MultiKalmanRobotMap.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of classes and \c RemoteMultiKalmanRobotMap.
 */

#include "MultiKalmanRobotMap.h"
#include "Tools/Debugging/DebugDrawings.h"


//MARK: RemoteMultiKalmanRobotMap

RemoteRobotMapHypothesis* MultiKalmanRobotMap::sensorUpdate(const Vector2f& measuredPosition,
    float measuredDistance,
    RobotEstimate::RobotType desiredRobotType,
    unsigned timestamp,
    float perceptValidity,
    float minDistanceForNewHypothesis,
    float initialValidityForNewHypothesis,
    const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
    int playerNumber,
    const typename RemoteRobotMapHypothesis::TeammateInfo teammate)

{
  // Reset index of best hypothesis. This must be recalculated each frame.
  resetBestHypothesisIndex();

  // Find the hypothesis which is nearest to the measurement.
  float distance;
  RemoteRobotMapHypothesis* nearestHypothesis = findNearestHypothesis(measuredPosition, distance, desiredRobotType);

  if (nearestHypothesis != nullptr && distance < minDistanceForNewHypothesis)
  {
    // TODO: make parameter
    float measurementNoiseFactor = measuredDistance / 1000.f < 1.f ? 1.f : measuredDistance / 1000.f; // convert distance to m
    // Set measurementNoiseMatrix according to the direction from robot to measurement position.
    // In this direction (^= distance) the noise is set to a higher value than in the orthogonal direction (^= angle).
    float max = static_cast<float>(nearestHypothesis->kalman.matrices.noise.maxMeasurementNoise);
    max *= measurementNoiseFactor;
    nearestHypothesis->kalman.matrices.noise.measurementNoiseMatrix = Covariance::create((Vector2f() << max, max / 10.f).finished(), measuredPosition.angle()).cast<double>();

    // Correct kalman filter with the perception.
    nearestHypothesis->sensorUpdate(measuredPosition, timestamp, perceptValidity);
  }
  else
  {
    // Add new hypothesis if all existing ones are too far.
    m_hypotheses.push_back(RemoteRobotMapHypothesis(kalmanNoiseMatrices, initialValidityForNewHypothesis, timestamp, perceptValidity, measuredPosition, getPerceptDuration()));
    nearestHypothesis = &m_hypotheses.back();
  }


  // Add a teammate label to the hypothesis.
  nearestHypothesis->addTeammateInfo(playerNumber, teammate);

  nearestHypothesis->addPerceptRobotType(desiredRobotType, timestamp);

  return nearestHypothesis;
}

RemoteRobotMapHypothesis* MultiKalmanRobotMap::findNearestHypothesis(const Vector2f& measuredPosition, float& distance, RobotEstimate::RobotType desiredRobotType)
{
  RemoteRobotMapHypothesis* nearestHypothesis = nullptr;
  float minDistance = -1;

  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    // Only consider hypotheses with the correct robot type.
    if (!m_hypotheses[i].matchRobotType(desiredRobotType))
      continue;

    // Calculate distance from the perception to the current hypothesis.
    float currentDistance = Geometry::distance(measuredPosition, m_hypotheses[i].kalman.position());

    // Check whether the current distance is shorter than the minimum of all
    // previous hypotheses.
    if (minDistance < 0 || currentDistance < minDistance)
    {
      nearestHypothesis = &m_hypotheses[i];
      minDistance = currentDistance;
    }
  }

  distance = minDistance;
  return nearestHypothesis;
}

void MultiKalmanRobotMap::updateRobotTypes(float minPerceptsPerSecond, float maxPerceptsPerSecond, unsigned timestamp)
{
  for (size_t i = 0; i < m_hypotheses.size(); i++)
    m_hypotheses[i].updateRobotType(minPerceptsPerSecond, maxPerceptsPerSecond, timestamp);
}

void MultiKalmanRobotMap::draw() const
{
  const static ColorRGBA teammateColor = ColorRGBA(150, 255, 0); // light green
  const static ColorRGBA opponentColor = ColorRGBA::red;
  const static ColorRGBA unknownColor = ColorRGBA::black;
  const float size = 50.f;

  DEBUG_DRAWING("module:KalmanRobotMapProvider:hypotheses", "drawingOnField")
  {
    for (const auto& hypothesis : m_hypotheses)
    {
      ColorRGBA color = (hypothesis.lastValidRobotType() == RobotEstimate::teammateRobot) ? teammateColor
          : (hypothesis.lastValidRobotType() == RobotEstimate::opponentRobot)
          ? opponentColor
          : unknownColor;
      float x = hypothesis.kalman.positionX();
      float y = hypothesis.kalman.positionY();
      float rot = hypothesis.kalman.velocity().angle();
      FILLED_TRIANGLE("module:KalmanRobotMapProvider:hypotheses", x, y, rot, 10, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
      ARROW("module:KalmanRobotMapProvider:hypotheses", x, y, x + hypothesis.kalman.velocityX(), y + hypothesis.kalman.velocityY(), 10, Drawings::solidPen, color);
      std::stringstream ss;
      ss << roundf(hypothesis.validity * 100.f) / 100.f;
      DRAWTEXT("module:KalmanRobotMapProvider:validity", x + 70, y, 75, color, ss.str().c_str());
      COVARIANCE2D("module:KalmanRobotMapProvider:covariances", hypothesis.kalman.covarianceMatrix.cast<float>(), hypothesis.kalman.position());
    }
  }
}
