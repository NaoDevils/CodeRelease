/**
 * \file RemoteKalmanPositionHypothesis.cpp
 * \author <a href="mailto:heiner.walter@tu-dortmund.de">Heiner Walter</a>
 *
 * Implementation of class \c RemoteKalmanPositionHypothesis.
 */

#include "RemoteKalmanPositionHypothesis.h"

#include <algorithm> // std::find (needed for compiling Nao)


//MARK: Labeling methods

void RemoteKalmanPositionHypothesis::addTeammateInfo(int playerNumber, const TeammateInfo& newTeammate)
{
  // Search for playerNumber in the current map of teammates.
  bool alreadyInMap = (m_teammates.find(playerNumber) != m_teammates.end());

  if (!alreadyInMap)
  {
    m_teammates.insert(std::pair<int, TeammateInfo>(playerNumber, newTeammate));
  }
  else
  {
    TeammateInfo old = m_teammates[playerNumber];
    if (newTeammate.timeWhenLastUpdated >= old.timeWhenLastUpdated)
      m_teammates[playerNumber] = newTeammate;
  }
}

void RemoteKalmanPositionHypothesis::addTeammates(const std::map<int, RemoteKalmanPositionHypothesis::TeammateInfo>& newTeammates)
{
  for (const auto& t : newTeammates)
    addTeammateInfo(t.first, t.second);
}

unsigned RemoteKalmanPositionHypothesis::timeWhenUpdatedByTeammate(int playerNumber) const
{
  // Search for playerNumber in the current map of teammates.
  bool inMap = (m_teammates.find(playerNumber) != m_teammates.end());

  if (!inMap)
    return 0; // not updated by this player
  return m_teammates.find(playerNumber)->second.timeWhenLastUpdated;
}

std::string RemoteKalmanPositionHypothesis::teammatesString() const
{
  // Concatenate all labels to one string.
  std::stringstream ss;
  bool first = true;
  for (const auto& t : m_teammates)
  {
    if (!first)
      ss << ",";
    first = false;
    ss << t.first;
  }
  return ss.str();
}


//MARK: Helper Methods

void RemoteKalmanPositionHypothesis::merge(const RemoteKalmanPositionHypothesis& source)
{
  // Call method from base class.
  KalmanPositionHypothesis::merge(source);
  // Add all labels of the merged hypothesis into this one.
  addTeammates(source.teammates());
}
