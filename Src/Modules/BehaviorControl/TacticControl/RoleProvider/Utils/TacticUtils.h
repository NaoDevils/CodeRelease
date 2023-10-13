#pragma once

#include "MathUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/Ranges/Cone.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Geometry.h"

class TacticUtils
{

public:
  [[nodiscard]] static Cone getDefenseCone(const Angle defenseWidth, const Vector2f& ballPosition, const FieldDimensions& theFieldDimensions)
  {
    ASSERT(0_deg < defenseWidth && defenseWidth < 360_deg);

    const Angle step = defenseWidth / 2;
    Angle leftAngle;
    Angle rightAngle;

    const bool ballOnOwnSide = ballPosition.x() < 0.f;
    if (ballOnOwnSide)
    {
      const Vector2f ownGoalCenter = FieldUtils::getOwnGoalCenter(theFieldDimensions);
      const Angle ownGoalCenterAngle = (ownGoalCenter - ballPosition).angle();
      leftAngle = Angle(ownGoalCenterAngle + step).normalize();
      rightAngle = Angle(ownGoalCenterAngle - step).normalize();

      const Vector2f ownLeftGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal};
      const Vector2f ownRightGoalPost = {theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal};
      const Angle ownLeftGoalPostAngle = (ownLeftGoalPost - ballPosition).angle();
      const Angle ownRightGoalPostAngle = (ownRightGoalPost - ballPosition).angle();
      const auto [ll, lr] = MathUtils::getLeftAndRightAngle(leftAngle, ownRightGoalPostAngle);
      const auto [rl, rr] = MathUtils::getLeftAndRightAngle(rightAngle, ownLeftGoalPostAngle);
      leftAngle = ll;
      rightAngle = rr;
    }
    else
    {
      rightAngle = 180_deg - step;
      leftAngle = -180_deg + step;
    }

    return {leftAngle, rightAngle};
  }
};
