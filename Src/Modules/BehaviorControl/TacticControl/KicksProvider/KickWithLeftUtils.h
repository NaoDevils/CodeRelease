#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/Cone.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/SelectedFoot.h"
#include "Tools/Math/Geometry.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/KickDrawings.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/TacticUtils.h"

class KickWithLeftUtils
{
public:
  static SelectedFoot getSelectedFootBasedOnDontRunInto(const Vector2f& ballPosition, const Vector2f& targetPosition, const Vector2f& opponentPosition)
  {
    const Pose2f ballPose = Pose2f((targetPosition - ballPosition).angle(), ballPosition);
    const Angle ballToOpponentAngle = Transformation::fieldToRobot(ballPose, opponentPosition).angle();

    const Angle pickSideToNotRunIntoOpponentAngle = 55_deg;
    if (ballToOpponentAngle > pickSideToNotRunIntoOpponentAngle)
    {
      return SelectedFoot::LEFT;
    }
    if (ballToOpponentAngle < -pickSideToNotRunIntoOpponentAngle)
    {
      return SelectedFoot::RIGHT;
    }
    return SelectedFoot::NONE;
  }

  static SelectedFoot getSelectedFootBasedOnMaximumBlock(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, const Vector2f& opponentPosition, const Cone& defenseCone)
  {
    const Pose2f ballPose = Pose2f((targetPosition - ballPosition).angle(), ballPosition);
    const Angle ballToOpponentAngle = Transformation::fieldToRobot(ballPose, opponentPosition).angle();

    const Angle pickSideToNotRunIntoOpponentAngle = 55_deg;
    if (ballToOpponentAngle > pickSideToNotRunIntoOpponentAngle)
    {
      return SelectedFoot::LEFT;
    }
    if (ballToOpponentAngle < -pickSideToNotRunIntoOpponentAngle)
    {
      return SelectedFoot::RIGHT;
    }
    return SelectedFoot::NONE;
  }

  static SelectedFoot getKickWithLeftBasedOn1v1(const Vector2f& ballPosition, const Vector2f& targetPosition, const Vector2f& opponentPosition)
  {
    const Pose2f ballPose = Pose2f((targetPosition - ballPosition).angle(), ballPosition);
    const Angle ballToOpponentAngle = Transformation::fieldToRobot(ballPose, opponentPosition).angle();

    const Angle pickSideToNotRunIntoOpponentAngle = 55_deg;
    if (ballToOpponentAngle > pickSideToNotRunIntoOpponentAngle)
    {
      OUTPUT_TEXT("dontRunInto: left");
      return SelectedFoot::LEFT;
    }
    if (ballToOpponentAngle < -pickSideToNotRunIntoOpponentAngle)
    {
      OUTPUT_TEXT("dontRunInto: right");
      return SelectedFoot::RIGHT;
    }

    const Angle pickSideToKickBySideOpponentAngle = 10_deg;
    if (ballToOpponentAngle > pickSideToKickBySideOpponentAngle)
    {
      OUTPUT_TEXT("kickBySide: right");
      return SelectedFoot::RIGHT;
    }
    if (ballToOpponentAngle < -pickSideToKickBySideOpponentAngle)
    {
      OUTPUT_TEXT("kickBySide: left");
      return SelectedFoot::LEFT;
    }

    OUTPUT_TEXT("none");
    return SelectedFoot::NONE;
  }

  static bool getKickWithLeftIfOnRightSide(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, SelectedFoot currentSelectedFoot)
  {
    return !getKickWithLeftIfOnLeftSide(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  }

  static bool getKickWithLeftIfOnLeftSide(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, SelectedFoot currentSelectedFoot)
  {
    float hysteresis;
    switch (currentSelectedFoot)
    {
    case SelectedFoot::LEFT:
      hysteresis = -202500.f; // 450 * 450 since it has to be squared, so 50cm in each direction
      break;
    case SelectedFoot::NONE:
      hysteresis = 0.f;
      break;
    case SelectedFoot::RIGHT:
      hysteresis = 202500.f;
      break;
    default:
      throw std::exception();
    }
    return !Geometry::isPointLeftOfLine(playerPose.translation, ballPosition, targetPosition, hysteresis);
  }

  static bool getKickWithLeftIfRightFootIsClosest(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, SelectedFoot currentSelectedFoot)
  {
    return !getKickWithLeftIfLeftFootIsClosest(playerPose, ballPosition, targetPosition, currentSelectedFoot);
  }

  static bool getKickWithLeftIfLeftFootIsClosest(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, SelectedFoot currentSelectedFoot)
  {
    if (Geometry::distance(playerPose.translation, ballPosition) < 750.f)
    {
      const Vector2f direction = Vector2f(targetPosition - ballPosition).rotateLeft();
      const Angle angleDiff = MathUtils::getLargerMinusSmallerAngleDiff(playerPose.rotation, direction.angle());
      float hysteresisMultiplier;
      switch (currentSelectedFoot)
      {
      case SelectedFoot::LEFT:
        hysteresisMultiplier = 1.05f;
        break;
      case SelectedFoot::NONE:
        hysteresisMultiplier = 1.f;
        break;
      case SelectedFoot::RIGHT:
        hysteresisMultiplier = 0.95f;
        break;
      default:
        throw std::exception();
      }
      const Angle angleDiffWithHysteresis = angleDiff * hysteresisMultiplier;
      return angleDiffWithHysteresis > 90_deg;
    }
    else
    {
      float hysteresis;
      switch (currentSelectedFoot)
      {
      case SelectedFoot::LEFT:
        hysteresis = 202500.f; // 450 * 450 since it has to be squared, so 45cm in each direction
        break;
      case SelectedFoot::NONE:
        hysteresis = 0.f;
        break;
      case SelectedFoot::RIGHT:
        hysteresis = -202500.f;
        break;
      default:
        throw std::exception();
      }
      return Geometry::isPointLeftOfLine(playerPose.translation, ballPosition, targetPosition, hysteresis);
    }
  }
};
