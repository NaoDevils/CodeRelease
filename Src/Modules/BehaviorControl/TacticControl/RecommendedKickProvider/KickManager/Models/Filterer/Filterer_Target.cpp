#include "Tools/Math/Geometry.h"
#include "Filterer.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"

Filterer& Filterer::filterTooHighRotationToKick(const Angle currentRotation, const Angle tooHighRotation)
{
  tooHighRotationToKickFilter = [currentRotation, tooHighRotation](const SelectableTarget& selectableTarget)
  {
    const Angle rotationDiff = MathUtils::getLargerMinusSmallerAngleDiff(selectableTarget.selectableKick.kick->optAngle, currentRotation);
    return rotationDiff > tooHighRotation;
  };
  return *this;
}

Filterer& Filterer::filterInFrontOfOwnGoal(const FieldDimensions& theFieldDimensions)
{
  inFrontOfOwnGoalFilter = [&theFieldDimensions](const SelectableTarget& selectableTarget)
  {
    const float x = selectableTarget.target.x();
    const float y = selectableTarget.target.y();

    if (x > (theFieldDimensions.xPosOwnGroundline + theFieldDimensions.xPosOwnGoalArea) / 2)
    {
      return false;
    }
    if (y > theFieldDimensions.yPosLeftGoal + theFieldDimensions.goalPostRadius)
    {
      return false;
    }
    if (theFieldDimensions.yPosRightGoal - theFieldDimensions.goalPostRadius > y)
    {
      return false;
    }
    return true;
  };
  return *this;
}

Filterer& Filterer::filterTooFarBack(float minX)
{
  tooFarBackFilter = [minX](const SelectableTarget& selectableTarget)
  {
    return selectableTarget.target.x() < minX;
  };
  return *this;
}

Filterer& Filterer::filterLeft(float maxY)
{
  leftFilter = [maxY](const SelectableTarget& selectableTarget)
  {
    return selectableTarget.target.y() > maxY;
  };
  return *this;
}

Filterer& Filterer::filterRight(float minY)
{
  rightFilter = [minY](SelectableTarget& selectableTarget)
  {
    return selectableTarget.target.y() < minY;
  };
  return *this;
}

Filterer& Filterer::filterLeftOfLine(const Vector2f& linePoint1, const Vector2f& linePoint2)
{
  leftOfLineFilter = [linePoint1, linePoint2](const SelectableTarget& selectableTarget)
  {
    return Geometry::isPointLeftOfLine(selectableTarget.target, linePoint1, linePoint2);
  };
  return *this;
}

Filterer& Filterer::filterCloseToFieldLine(const FieldDimensions& theFieldDimensions)
{
  closeToFieldLineFilter = [&theFieldDimensions](const SelectableTarget& selectableTarget)
  {
    const float MIN_DISTANCE_TO_FIELD_LINE = 400.f;
    const Angle MIN_UNPROBLEMATIC_ANGLE = 5_deg;

    const bool leftCloseToFieldLine = selectableTarget.target.y() > theFieldDimensions.yPosLeftSideline - MIN_DISTANCE_TO_FIELD_LINE;
    const bool rightCloseToFieldLine = theFieldDimensions.yPosRightSideline + MIN_DISTANCE_TO_FIELD_LINE > selectableTarget.target.y();
    const Angle kickAngle = (selectableTarget.target - selectableTarget.selectableDirection.ballPosition).angle();
    if (leftCloseToFieldLine && (kickAngle < -180_deg + MIN_UNPROBLEMATIC_ANGLE || -MIN_UNPROBLEMATIC_ANGLE < kickAngle))
    {
      return true;
    }
    if (rightCloseToFieldLine && (kickAngle > 180_deg - MIN_UNPROBLEMATIC_ANGLE || MIN_UNPROBLEMATIC_ANGLE > kickAngle))
    {
      return true;
    }
    return false;
  };
  return *this;

  /*
   * TODO Also filter close to front and back field lines
   *
  closeToFieldLineFilter = [&theFieldDimensions](const SelectableTarget& selectableTarget)
  {
    const float MIN_DISTANCE_TO_FIELD_LINE = 250.f;
    const Angle MIN_UNPROBLEMATIC_ANGLE = 5_deg;
    const Angle MIN_GOAL_POST_TO_BALL_ANGLE = 100_deg;

    const Angle shotAngle = (selectableTarget.target - selectableTarget.ballPosition).angle();

    const bool leftCloseToFieldLine = selectableTarget.target.y() > theFieldDimensions.yPosLeftSideline - MIN_DISTANCE_TO_FIELD_LINE;
    const bool shotAlongLeftFieldLine = shotAngle < -180_deg + MIN_UNPROBLEMATIC_ANGLE || -MIN_UNPROBLEMATIC_ANGLE < shotAngle;
    if (leftCloseToFieldLine && shotAlongLeftFieldLine)
    {
      return true;
    }

    const bool rightCloseToFieldLine = theFieldDimensions.yPosRightSideline + MIN_DISTANCE_TO_FIELD_LINE > selectableTarget.target.y();
    const bool shotAlongRightFieldLine = rightCloseToFieldLine && (shotAngle > 180_deg - MIN_UNPROBLEMATIC_ANGLE || MIN_UNPROBLEMATIC_ANGLE > shotAngle);
    if (rightCloseToFieldLine && shotAlongRightFieldLine)
    {
      return true;
    }

    const bool opponentSide = selectableTarget.target.x() > 0.f;
    const bool leftSide = selectableTarget.target.y() > 0.f;

    const Vector2f goalPost = opponentSide
        ? (leftSide ? FieldUtils::getOpponentGoalLeftPost(theFieldDimensions) : FieldUtils::getOpponentGoalRightPost(theFieldDimensions))
        : (leftSide ? FieldUtils::getOwnGoalLeftPost(theFieldDimensions) : FieldUtils::getOwnGoalRightPost(theFieldDimensions));
    const Angle goalPostToBallAngle = (selectableTarget.ballPosition - goalPost).angle();

    if (opponentSide)
    {
      bool opponentSideCloseToFieldLine;
      if (leftSide)
      {
        opponentSideCloseToFieldLine = goalPostToBallAngle > 0_deg && goalPostToBallAngle < MIN_GOAL_POST_TO_BALL_ANGLE;
      }
      else
      {
        opponentSideCloseToFieldLine = goalPostToBallAngle < 0_deg && goalPostToBallAngle > -MIN_GOAL_POST_TO_BALL_ANGLE;
      }
    }
    else
    {
      bool ownSideCloseToFieldLine;
      if (leftSide)
      {
        ownSideCloseToFieldLine = goalPostToBallAngle < 180_deg && goalPostToBallAngle > 180_deg - MIN_GOAL_POST_TO_BALL_ANGLE;
      }
      else
      {
        ownSideCloseToFieldLine = goalPostToBallAngle > -180_deg && goalPostToBallAngle < -MIN_GOAL_POST_TO_BALL_ANGLE;
      }
    }






    if (selectableTarget.target.y() > 0.f)
    {
      const Angle leftGoalPostToBallAngle = (selectableTarget.ballPosition - goalPost).angle();
      const bool leftGoalCloseToFieldLine = leftGoalPostToBallAngle > 0_deg && leftGoalPostToBallAngle < MIN_GOAL_POST_TO_BALL_ANGLE;
    }
    else
    {
      const Vector2f rightGoalPost = selectableTarget.target.x() < 0.f ? FieldUtils::getOwnGoalRightPost(theFieldDimensions) : FieldUtils::getOpponentGoalRightPost(theFieldDimensions);
      const Angle rightGoalPostToBallAngle = (selectableTarget.ballPosition - rightGoalPost).angle();
      if (rightGoalPostToBallAngle < 0_deg && rightGoalPostToBallAngle > -MIN_GOAL_POST_TO_BALL_ANGLE)
      {
        return true;
      }
    }

    return false;
  };
  return *this;
  */
}
