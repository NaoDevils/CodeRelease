#include "HeadControl.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Math/Transformation.h"

void HeadControl::update(HeadAngleRequest& headAngleRequest)
{
  tiltLimits.min = transformHeadPosition(maxTiltObject, false, -theCameraInfoUpper.openingAngleHeight / 2.f).y();
  tiltLimits.max = transformHeadPosition(minTiltObject, true, theCameraInfo.openingAngleHeight / 2.f).y();

  headAngleRequest.speed = defaultSpeed;
  if (theRobotInfo.number % 2 == 0)
    headAngleRequest.speed = 15_deg;

  const std::vector<Vector2a> cameraPositions = getNextTarget();

  const Vector2a currentHeadAngle(theJointSensorData.angles[Joints::Joint::headYaw], theJointSensorData.angles[Joints::Joint::headPitch]);

  if (lastCameraPosition >= cameraPositions.size())
    lastCameraPosition = cameraPositions.size() - 1;

  Vector2a targetPosition = cameraPositions[lastCameraPosition];
  for (size_t i = 0; i < cameraPositions.size(); ++i)
  {
    const Vector2a& cameraPosition = cameraPositions[i];
    if ((cameraPosition - currentHeadAngle).norm() < (targetPosition - currentHeadAngle).norm() - 5_deg)
    {
      lastCameraPosition = i;
      targetPosition = cameraPosition;
    }
  }

  //if (theHeadPOIList.type == HeadPOIList::Type::sweep && theHeadPOIList.targets.size() > 1)
  //{
  //  const Vector2a moveHeadDirection = targetPosition - currentHeadAngle;
  //  const Vector2a adjustedTargetPosition = targetPosition - moveHeadDirection.normalized() * sweepAngleReached;
  //  headAngleRequest.pan = adjustedTargetPosition.x();
  //  headAngleRequest.tilt = adjustedTargetPosition.y();
  //}
  //else
  {
    headAngleRequest.pan = targetPosition.x();
    headAngleRequest.tilt = targetPosition.y();
  }
}

std::vector<Vector2a> HeadControl::getNextTarget()
{
  std::vector<std::vector<Vector2a>> targetsToAngles;
  for (const HeadPOIList::Target& target : theHeadPOIList.targets)
  {
    std::vector<Vector2a> headAngles = getHeadAngles(target);

    for (Vector2a& headAngle : headAngles)
      headAngle.y() = tiltLimits.limit(headAngle.y());

    targetsToAngles.emplace_back(std::move(headAngles));
  }

  const auto isLess = [](const std::vector<Vector2a>& a, const std::vector<Vector2a>& b)
  {
    return a.front().x() < b.front().x() || (a.front().x() == b.front().x() && a.front().y() < b.front().y());
  };
  std::sort(targetsToAngles.begin(), targetsToAngles.end(), isLess);

  if (currentTarget >= targetsToAngles.size())
    currentTarget = targetsToAngles.size() - 1;

  const Angle angleThreshold = theHeadPOIList.type == HeadPOIList::Type::sweep ? sweepAngleReached : focusAngleReached;
  const Vector2a currentHeadAngle(theJointSensorData.angles[Joints::Joint::headYaw], theJointSensorData.angles[Joints::Joint::headPitch]);


  const size_t lastTarget = currentTarget;
  std::vector<Vector2a> visibleTargets;
  for (size_t i = 0; i < targetsToAngles.size(); ++i)
  {
    const std::vector<Vector2a>& targetAngles = targetsToAngles[currentTarget];
    for (const Vector2a& targetAngle : targetAngles)
    {
      if ((targetAngle - currentHeadAngle).norm() < angleThreshold)
      {
        visibleTargets.push_back(targetAngle);

        if (targetsToAngles.size() > 1)
        {
          currentTarget += movingLeft ? -1 : +1;

          if (currentTarget >= targetsToAngles.size())
          {
            movingLeft = !movingLeft;
            currentTarget += movingLeft ? -2 : +2;
          }
        }
        break;
      }
    }
  }

  if (currentTarget == lastTarget && visibleTargets.size() > 1)
  {
    Vector2a meanTarget(0_deg, 0_deg);

    for (const Vector2a& visibleTarget : visibleTargets)
      meanTarget += visibleTarget;

    meanTarget /= static_cast<float>(visibleTargets.size());

    return {meanTarget};
  }
  else
  {
    return targetsToAngles[currentTarget];
  }
}

std::vector<Vector2a> HeadControl::getHeadAngles(const HeadPOIList::Target& target) const
{
  std::vector<Vector2a> ret;
  switch (target.type)
  {
  case HeadPOIList::Target::Type::angle:
    ret.emplace_back(target.position.cast<Angle>());
    break;
  case HeadPOIList::Target::Type::ball:
  {
    const Vector2f ball = theBallSymbols.ballPositionRelativeWOPreview + theBallSymbols.ballVelocityRelativeWOPreview / 2.f;
    const Vector3f ball3f(ball.x(), ball.y(), 50.f);

    if (ball.norm() <= maxDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(ball3f, true) + target.position.cast<Angle>());
    if (ball.norm() >= minDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(ball3f, false) + target.position.cast<Angle>());
  }
  break;
  case HeadPOIList::Target::Type::remoteBall:
  {
    const Vector2f ball = theRemoteBallModel.position + theRemoteBallModel.velocity / 2.f;
    Vector2f remoteBallPositionRelative = Transformation::fieldToRobot(theRobotPose, ball);
    const Vector3f ball3f(remoteBallPositionRelative.x(), remoteBallPositionRelative.y(), 50.f);

    if (ball.norm() <= maxDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(ball3f, true) + target.position.cast<Angle>());
    if (ball.norm() >= minDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(ball3f, false) + target.position.cast<Angle>());
  }
  break;
  case HeadPOIList::Target::Type::fieldPosition:
  {
    Vector3f targetRobot;
    targetRobot << Transformation::fieldToRobot(theRobotPose, target.position), 0.f;

    if (targetRobot.norm() <= maxDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(targetRobot, true));
    if (targetRobot.norm() >= minDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(targetRobot, false));
  }
  break;
  case HeadPOIList::Target::Type::robot:
  {
    Vector3f targetRobot;
    targetRobot << Transformation::fieldToRobot(theRobotPose, target.position), 200.f; // look at ~20 cm too see the whole robot including the feet

    if (targetRobot.norm() <= maxDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(targetRobot, true));
    if (targetRobot.norm() >= minDistanceForLowerCamera)
      ret.emplace_back(transformHeadPosition(targetRobot, false));
  }
  break;
  default:
    ASSERT(false);
  }
  return ret;
}

Vector2a HeadControl::transformHeadPosition(const Vector3f& headpos, bool lowerCamera, Angle imageTilt, Angle imagePan) const
{
  Vector3f hip2Target = theTorsoMatrix.inverse() * headpos;
  return InverseKinematic::calcHeadJoints(hip2Target, theRobotDimensions, lowerCamera, theCameraCalibration, imageTilt, imagePan);
}

MAKE_MODULE(HeadControl, behaviorControl)
