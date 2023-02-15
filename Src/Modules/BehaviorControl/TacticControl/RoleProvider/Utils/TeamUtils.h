#pragma once

#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/Infrastructure/TeammateData.h"
#include <optional>

class TeamUtils
{
public:
  static std::optional<Vector2f> getRealBallChaserPosition(const BallChaserDecision& theBallChaserDecision, const TeammateData& theTeammateData)
  {
    Vector2f realBallChaserPosition;
    bool foundBallchaser = false;
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.number == theBallChaserDecision.playerNumberToBall)
      {
        realBallChaserPosition = mate.pose.translation;
        foundBallchaser = true;
        break;
      }
    }
    if (foundBallchaser)
    {
      return std::optional<Vector2f>(realBallChaserPosition);
    }
    else
    {
      return std::optional<Vector2f>(); // TODO. Maybe use as backup?  theBallchaser.optPosition.translation;
    }
  }

  static std::optional<Vector2f> getTeammatePosition(const TeammateData& theTeammateData, BehaviorData::RoleAssignment role)
  {
    Vector2f teammatePosition;
    bool foundTeammate = false;

    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.behaviorData.role == role)
      {
        teammatePosition = mate.pose.translation;
        foundTeammate = true;
        break;
      }
    }
    if (foundTeammate)
    {
      return std::optional<Vector2f>(teammatePosition);
    }
    else
    {
      return std::optional<Vector2f>();
    }
  }

  static std::optional<Vector2f> getDesiredTeammatePosition(const TeammateData& theTeammateData, BehaviorData::RoleAssignment role)
  {
    Vector2f desiredPosition;
    bool foundTeammate = false;

    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.behaviorData.role == role)
      {
        desiredPosition = mate.walkRequest.request.translation;
        foundTeammate = true;
        break;
      }
    }
    if (foundTeammate)
    {
      return std::optional<Vector2f>(desiredPosition);
    }
    else
    {
      return std::optional<Vector2f>();
    }
  }

  static std::vector<Teammate> getPlayingTeammates(const TeammateData& theTeammateData)
  {
    std::vector<Teammate> playingTeammates;
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.status >= Teammate::ACTIVE) // not penalized
      {
        playingTeammates.push_back(mate);
      }
    }
    return playingTeammates;
  }
};
