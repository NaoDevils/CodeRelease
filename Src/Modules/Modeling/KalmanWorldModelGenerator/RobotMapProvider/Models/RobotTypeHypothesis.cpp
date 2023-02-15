/**
 * \file RobotTypeHypothesis.cpp
 * \author <a href="mailto:dino.menges@tu-dortmund.de">Dino Menges</a>
 *
 * Implementation of class RobotTypeHypothesis.
 */

#include "RobotTypeHypothesis.h"

bool RobotTypeHypothesis::matchRobotType(RobotEstimate::RobotType otherRobotType) const
{
  // Return true if one of the robot types is unknown.
  if (m_robotType == RobotEstimate::RobotType::unknownRobot || otherRobotType == RobotEstimate::RobotType::unknownRobot)
    return true;
  // Compare robot types
  return m_robotType == otherRobotType;
}

void RobotTypeHypothesis::addPerceptRobotType(RobotEstimate::RobotType preceptRobotType, unsigned timestamp)
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

void RobotTypeHypothesis::updateRobotType(float minPerceptsPerSecond, float maxPerceptsPerSecond, unsigned timestamp)
{
  float unknownThreshold = static_cast<float>(minPerceptsPerSecond) / static_cast<float>(maxPerceptsPerSecond);
  // Update PPS buffers
  m_perceptsPerSecond_opponent.updateCurrentTime(timestamp);
  m_perceptsPerSecond_teammate.updateCurrentTime(timestamp);

  // The PPS based robot type validity in range [-1, 1] indicates the team of this robot hypothesis.
  // -1: teammate; 0: unknown, 1: opponent
  float pps = m_perceptsPerSecond_opponent.pps() * ROBOT_TYPE_VALIDITY_OPPONENT + m_perceptsPerSecond_teammate.pps() * ROBOT_TYPE_VALIDITY_TEAMMATE;
  m_robotTypeValidity = pps / static_cast<float>(maxPerceptsPerSecond);

  // Cut validity to interval [-1,1].
  m_robotTypeValidity = m_robotTypeValidity > 1.f ? 1.f : m_robotTypeValidity < -1.f ? -1.f : m_robotTypeValidity;

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

void RobotTypeHypothesis::merge(const RobotTypeHypothesis& other)
{
  // Merge percepts
  m_perceptsPerSecond_opponent.addPercepts(other.m_perceptsPerSecond_opponent);
  m_perceptsPerSecond_teammate.addPercepts(other.m_perceptsPerSecond_teammate);

  // Merge validities
  m_robotTypeValidity += other.m_robotTypeValidity;
  m_robotTypeValidity /= 2;
}
