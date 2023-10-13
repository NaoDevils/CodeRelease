#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/SelectedFoot.h"
#include "Tools/Math/Geometry.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/MathUtils.h"

class KickUtils
{
public:
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

  static bool getKickWithLeftIfInnerFoot(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, SelectedFoot currentSelectedFoot)
  {
    const Vector2f direction = Vector2f(targetPosition - ballPosition).rotateLeft();
    const Geometry::Line kickDecisionLine = {ballPosition, direction};

    if (Geometry::distance(playerPose.translation, ballPosition) < 750.f)
    {
      const Angle angleDiff = MathUtils::getAngleDiff(playerPose.rotation, direction.angle());
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
        hysteresis = 202500.f; // 450 * 450 since it has to be squared, so 50cm in each direction
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
