#pragma once

#include "Representations/BehaviorControl/BallChaserDecision.h"
#include "Representations/Infrastructure/TeammateData.h"
#include <optional>

class TeamUtils
{
public:
  static std::optional<Vector2f> getRealBallChaserPosition(const BallChaserDecision& theBallChaserDecision, const TeammateData& theTeammateData)
  {
    if (theTeammateData.myself.playerNumber == theBallChaserDecision.playerNumberToBall)
    {
      return theTeammateData.myself.robotPose.translation;
    }

    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.playerNumber == theBallChaserDecision.playerNumberToBall)
      {
        return mate.robotPose.translation;
      }
    }

    return {};
  }

  static std::optional<Vector2f> getTeammatePosition(const TeammateData& theTeammateData, BehaviorData::RoleAssignment role)
  {
    Vector2f teammatePosition;
    bool foundTeammate = false;

    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.behaviorData.role == role)
      {
        teammatePosition = mate.robotPose.translation;
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

  static std::vector<Teammate> getPlayingTeammates(const TeammateData& theTeammateData)
  {
    std::vector<Teammate> playingTeammates;
    for (auto& mate : theTeammateData.teammates)
    {
      if (mate.status >= TeammateReceived::Status::ACTIVE) // not penalized
      {
        playingTeammates.push_back(mate);
      }
    }
    return playingTeammates;
  }
};
