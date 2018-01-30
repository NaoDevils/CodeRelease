/**
 * \file MultiKalmanRobotMap.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of classes and \c RemoteMultiKalmanRobotMap.
 */

#include "MultiKalmanRobotMap.h"
#include "Tools/Debugging/DebugDrawings.h"


// MARK: MultiKalmanRobotMapHypothesis

bool MultiKalmanRobotMapHypothesis::matchRobotType(RobotEstimate::RobotType otherRobotType) const
{
  // Return true if one of the robot types is unknown.
  if (m_robotType == RobotEstimate::RobotType::unknownRobot ||
      otherRobotType == RobotEstimate::RobotType::unknownRobot) return true;
  // Compare robot types
  return m_robotType == otherRobotType;
}

void MultiKalmanRobotMapHypothesis::addPerceptRobotType(
  RobotEstimate::RobotType preceptRobotType,
  unsigned timestamp)
{
  if (preceptRobotType == RobotEstimate::RobotType::opponentRobot)
  {
    m_perceptsPerSecond_opponent.addPercept(timestamp);
  }
  else if (preceptRobotType == RobotEstimate::RobotType::teammateRobot)
  {
    m_perceptsPerSecond_teammate.addPercept(timestamp);
  }
}

void MultiKalmanRobotMapHypothesis::updateRobotType(
  size_t minPerceptsPerSecond,
  size_t maxPerceptsPerSecond,
  unsigned timestamp)
{
  float unknownThreshold = static_cast<float>(minPerceptsPerSecond) / static_cast<float>(maxPerceptsPerSecond);
  // Update PPS buffers
  m_perceptsPerSecond_opponent.updateCurrentTime(timestamp);
  m_perceptsPerSecond_teammate.updateCurrentTime(timestamp);
  
  // The PPS based robot type validity in range [-1, 1] indicates the team of this robot hypothesis.
  // -1: teammate; 0: unknown, 1: opponent
  float pps = m_perceptsPerSecond_opponent.pps() * ROBOT_TYPE_VALIDITY_OPPONENT
  + m_perceptsPerSecond_teammate.pps() * ROBOT_TYPE_VALIDITY_TEAMMATE;
  m_robotTypeValidity = pps / static_cast<float>(maxPerceptsPerSecond);
  
  // Cut validity to interval [-1,1].
  m_robotTypeValidity = m_robotTypeValidity >  1.f ?  1.f :
                        m_robotTypeValidity < -1.f ? -1.f :
                        m_robotTypeValidity;
  
  if (m_robotTypeValidity * ROBOT_TYPE_VALIDITY_OPPONENT > unknownThreshold)
  {
    m_robotType = RobotEstimate::RobotType::opponentRobot;
    m_lastRobotType = m_robotType;
  }
  else if (m_robotTypeValidity * ROBOT_TYPE_VALIDITY_TEAMMATE > unknownThreshold)
  {
    m_robotType = RobotEstimate::RobotType::teammateRobot;
    m_lastRobotType = m_robotType;
  }
  else
  {
    m_robotType = RobotEstimate::RobotType::unknownRobot;
  }
}


//MARK: RobotMapEntry

void updateRobotMapEntry(const MultiKalmanRobotMapHypothesis& hypothesis, RobotMapEntry& robot)
{
  robot.pose.translation = hypothesis.kalman.position(); // (in mm)
  // Get rotation of robot from its motion direction.
  // TODO: This is not the direction the robot is facing
  robot.pose.rotation = hypothesis.kalman.velocity().angle(); // (in radiant)
  robot.robotType = hypothesis.lastValidRobotType();
}


//MARK: RemoteMultiKalmanRobotMap

MultiKalmanRobotMapHypothesis* MultiKalmanRobotMap::sensorUpdate(
  const Vector2f& measuredPosition,
  float measuredDistance,
  RobotEstimate::RobotType desiredRobotType,
  unsigned timestamp,
  float perceptValidity,
  float minDistanceForNewHypothesis,
  float initialValidityForNewHypothesis,
  const KalmanPositionTracking2D<double>::KalmanMatrices::Noise& kalmanNoiseMatrices,
  int playerNumber,
  const typename MultiKalmanRobotMapHypothesis::TeammateInfo teammate)

{
  // Reset index of best hypothesis. This must be recalculated each frame.
  resetBestHypothesisIndex();
  
  // Find the hypothesis which is nearest to the measurement.
  float distance;
  MultiKalmanRobotMapHypothesis* nearestHypothesis = findNearestHypothesis(measuredPosition, distance, desiredRobotType);
  
  if (nearestHypothesis != nullptr && distance < minDistanceForNewHypothesis)
  {
    // TODO: make parameter
    float measurementNoiseFactor = measuredDistance / 1000.f < 1.f ? 1.f : measuredDistance / 1000.f; // convert distance to m
    // Set measurementNoiseMatrix according to the direction from robot to measurement position.
    // In this direction (^= distance) the noise is set to a higher value than in the orthogonal direction (^= angle).
    float max = static_cast<float>(nearestHypothesis->kalman.matrices.noise.maxMeasurementNoise);
    max *= measurementNoiseFactor;
    nearestHypothesis->kalman.matrices.noise.measurementNoiseMatrix =
    Covariance::create((Vector2f() << max, max / 10.f).finished(), measuredPosition.angle()).cast<double>();
    
    // Correct kalman filter with the perception.
    nearestHypothesis->sensorUpdate(measuredPosition, timestamp, perceptValidity);
  }
  else
  {
    // Add new hypothesis if all existing ones are too far.
    MultiKalmanRobotMapHypothesis newHypothesis(kalmanNoiseMatrices,
                                                initialValidityForNewHypothesis,
                                                timestamp,
                                                perceptValidity,
                                                measuredPosition);
    
    m_hypotheses.push_back(newHypothesis);
    nearestHypothesis = &m_hypotheses.back();
  }
  
  
  // Add a teammate label to the hypothesis.
  nearestHypothesis->addTeammateInfo(playerNumber, teammate);

  nearestHypothesis->addPerceptRobotType(desiredRobotType, timestamp);
  
  return nearestHypothesis;
}

MultiKalmanRobotMapHypothesis* MultiKalmanRobotMap::findNearestHypothesis(
  const Vector2f& measuredPosition, float& distance,
  RobotEstimate::RobotType desiredRobotType)
{
  MultiKalmanRobotMapHypothesis* nearestHypothesis = nullptr;
  float minDistance = -1;
  
  for (size_t i = 0; i < m_hypotheses.size(); i++)
  {
    // Only consider hypotheses with the correct robot type.
    if (!m_hypotheses[i].matchRobotType(desiredRobotType)) continue;
    
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

void MultiKalmanRobotMap::updateRobotTypes(size_t minPerceptsPerSecond,
                                           size_t maxPerceptsPerSecond,
                                           unsigned timestamp)
{
  for (size_t i = 0; i < m_hypotheses.size(); i++)
    m_hypotheses[i].updateRobotType(minPerceptsPerSecond,
                                    maxPerceptsPerSecond,
                                    timestamp);
}

void MultiKalmanRobotMap::draw() const
{
  const static ColorRGBA teammateColor = ColorRGBA(150, 255, 0); // light green
  const static ColorRGBA opponentColor = ColorRGBA::red;
  const static ColorRGBA unknownColor = ColorRGBA::black;
  const float size = 50.f;
  
  DEBUG_DRAWING("module:KalmanRobotMapProvider:hypotheses", "drawingOnField")
  {
    for (std::vector<MultiKalmanRobotMapHypothesis>::const_iterator i = m_hypotheses.begin(); i != m_hypotheses.end(); ++i)
    {
      ColorRGBA color = (i->lastValidRobotType() == RobotEstimate::teammateRobot) ? teammateColor :
                        (i->lastValidRobotType() == RobotEstimate::opponentRobot) ? opponentColor :
                        unknownColor;
      float x = i->kalman.positionX();
      float y = i->kalman.positionY();
      float rot = i->kalman.velocity().angle();
      FILLED_TRIANGLE("module:KalmanRobotMapProvider:hypotheses",
                      x, y, rot,
                      10, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
      ARROW("module:KalmanRobotMapProvider:hypotheses",
            x, y,
            x + i->kalman.velocityX(), y + i->kalman.velocityY(),
            10, Drawings::solidPen, color);
      std::stringstream ss; ss << roundf(i->validity*100.f)/100.f;
      DRAWTEXT("module:KalmanRobotMapProvider:validity", x + 70, y, 75, color, ss.str().c_str());
    }
  }
}
