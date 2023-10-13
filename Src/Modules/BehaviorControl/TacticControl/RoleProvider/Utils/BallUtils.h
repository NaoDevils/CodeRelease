#pragma once

#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/RoleSymbols/Center.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/RobotMap.h"
#include <Representations/Sensing/RobotModel.h>
#include "Representations/Configuration/FieldDimensions.h"

class BallUtils
{
public:
  static Vector2f getBallPosition(const BallSymbols& theBallSymbols, const FrameInfo& theFrameInfo, const RobotPoseAfterPreview& theRobotPoseAfterPreview)
  {
    // changes to the predicted position made this method useless. todo delete
    return theBallSymbols.ballPositionFieldPredicted;

    /*
    const float ONLY_PREDICTED_DISTANCE = 2000.f;
    const float ONLY_REAL_DISTANCE = 1000.f;

    if (theBallSymbols.timeSinceLastSeenByTeam == static_cast<int>(theFrameInfo.time)) // never seen -> assume center position
    {
      return Vector2f::Zero();
    }

    const float distanceToBall = Geometry::distance(theRobotPoseAfterPreview.translation, theBallSymbols.ballPositionField);
    if (distanceToBall < ONLY_REAL_DISTANCE)
    {
      return theBallSymbols.ballPositionField;
    }
    if (distanceToBall > ONLY_PREDICTED_DISTANCE)
    {
      return theBallSymbols.ballPositionFieldPredicted;
    }
    const float multiplier = (distanceToBall - ONLY_REAL_DISTANCE) / (ONLY_PREDICTED_DISTANCE - ONLY_REAL_DISTANCE);
    const Vector2f ballPosition = multiplier * theBallSymbols.ballPositionFieldPredicted + (1-multiplier) * theBallSymbols.ballPositionField;
    ASSERT(std::isnormal(ballPosition.x()) || ballPosition.x() == 0.0f);
    ASSERT(std::isnormal(ballPosition.y()) || ballPosition.y() == 0.0f);
    return ballPosition;
    */
  }
};
