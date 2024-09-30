#pragma once

#include <algorithm>
#include "TeamUtils.h"
#include "Tools/Math/Geometry.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"

class AvoidUtils
{
public:
  static void avoidPath(Vector2f& position, const Vector2f& startPosition, const Vector2f& targetPosition, const float avoidDistance, const bool avoidLeft)
  {
    // Avoid the path of the Ballchaser. Since Path is unstable we use a line
    Vector2f vectorBallChaserToPosition = startPosition - targetPosition;
    float distanceToBallChaserPath = Geometry::getDistanceToEdge(Geometry::Line(targetPosition, vectorBallChaserToPosition), position);
    if (distanceToBallChaserPath < avoidDistance)
    {
      if (avoidLeft)
        position.y() += (avoidDistance - distanceToBallChaserPath);
      else
        position.y() -= (avoidDistance - distanceToBallChaserPath);
    }
  }

  static void avoid(Vector2f& position, const Vector2f& avoidPosition, const float avoidDistance)
  {
    Vector2f vectorToPosition = (avoidPosition - position);
    float distanceToPosition = vectorToPosition.norm();
    if (distanceToPosition < avoidDistance)
    {
      position -= vectorToPosition.normalize(avoidDistance - distanceToPosition);
    }
  }

  // =====================================================================================================================

  static void avoidYOutside(PositioningSymbols& positioningSymbols, const FieldDimensions& theFieldDimensions)
  {
    positioningSymbols.optPosition.translation.y() = std::min(positioningSymbols.optPosition.translation.y(), theFieldDimensions.yPosLeftSideline);
    positioningSymbols.optPosition.translation.y() = std::max(positioningSymbols.optPosition.translation.y(), theFieldDimensions.yPosRightSideline);
  }

  static void avoidBall(PositioningSymbols& positioningSymbols, const Vector2f& ballPosition)
  {
    const float avoidBallDistance = 1000.f;
    avoid(positioningSymbols.optPosition.translation, ballPosition, avoidBallDistance);
  }

  static void avoidBallChaser(PositioningSymbols& positioningSymbols,
      const Ballchaser& theBallchaser,
      const BallChaserDecision& theBallChaserDecision,
      const BehaviorConfiguration& theBehaviorConfiguration,
      const TeammateData& theTeammateData,
      const bool avoidBallChaserLeft)
  {
    if (std::optional<Vector2f> realBallChaserPositionOptional = TeamUtils::getRealBallChaserPosition(theBallChaserDecision, theTeammateData))
    {
      Vector2f realBallChaserPosition = *realBallChaserPositionOptional;

      avoidPath(positioningSymbols.optPosition.translation, theBallchaser.optPosition.translation, realBallChaserPosition, theBehaviorConfiguration.behaviorParameters.positionConflictDistance, avoidBallChaserLeft);
    }
  }

  static void avoidPositionConflict(PositioningSymbols& positioningSymbols, const BehaviorConfiguration& theBehaviorConfiguration, const TeammateData& theTeammateData, const Vector2f& ballPosition)
  {
    // This method assumes that the desired position has already been calculated!!
    for (auto& mate : TeamUtils::getPlayingTeammates(theTeammateData))
    {
      avoid(positioningSymbols.optPosition.translation, mate.robotPose.translation, theBehaviorConfiguration.behaviorParameters.positionConflictDistance);
    }
  }
};
