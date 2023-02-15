#pragma once

#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"

#include <utility>
#include "Tools/Math/Eigen.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Models/DistanceRequirement.h"

class Kick
{
public:
  Kick(std::string name, const float minDistance, const float maxDistance, const bool distanceAdjustable, const float time, const float horizontalInaccuracy, const bool kickBlind, const Angle& optAngle, const float optXDistanceToBall, const float optYDistanceToBall)
      : minDistance(minDistance), maxDistance(maxDistance), distanceAdjustable(distanceAdjustable), time(time), verticalInaccuracy(maxDistance - minDistance),
        horizontalInaccuracy(horizontalInaccuracy), inaccuracy((verticalInaccuracy + horizontalInaccuracy * 1000.f / maxDistance) / 2), kickBlind(kickBlind), optAngle(optAngle),
        optXDistanceToBall(optXDistanceToBall), optYDistanceToBall(optYDistanceToBall), name(std::move(name))
  {
  }
  virtual ~Kick() = default;

  [[nodiscard]] float getMinDistance(const bool hysteresis, const bool biggerIsBetter) const
  {
    const float HYSTERESIS_DISTANCE = 300.f;
    return minDistance + (hysteresis ? HYSTERESIS_DISTANCE * (biggerIsBetter ? 1.f : -1.f) : 0.f);
  }
  [[nodiscard]] float getMaxDistance(const bool hysteresis, const bool biggerIsBetter) const
  {
    const float HYSTERESIS_DISTANCE = 300.f;
    return maxDistance + (hysteresis ? HYSTERESIS_DISTANCE * (biggerIsBetter ? 1.f : -1.f) : 0.f);
  }
  [[nodiscard]] float getTime(const bool hysteresis) const { return hysteresis ? time * 0.5f : time; }
  [[nodiscard]] std::string getName() const { return name; }
  [[nodiscard]] bool isDistanceAdjustable() const { return distanceAdjustable; }

  [[nodiscard]] float getInaccuracy() const { return inaccuracy; }

  [[nodiscard]] float getVerticalInaccuracy() const { return verticalInaccuracy; }

  [[nodiscard]] float getHorizontalInaccuracy() const { return horizontalInaccuracy; }

  [[nodiscard]] Vector2f getRealisticTarget(const Vector2f& from, const Vector2f& to) const
  {
    const Vector2f fromTo = (to - from);
    const Vector2f direction = fromTo.normalized();
    const float distance = fromTo.norm();
    const float realisticDistance = getRealisticDistance(distance, false, false);
    return from + direction * realisticDistance;
  }

  [[nodiscard]] float getRealisticDistance(float distance, const bool hysteresis, const bool biggerIsBetter) const
  {
    const float hysteresisMinDistance = getMinDistance(hysteresis, biggerIsBetter);
    const float hysteresisMaxDistance = getMaxDistance(hysteresis, biggerIsBetter);
    if (distanceAdjustable)
    {
      if (distance < hysteresisMinDistance)
      {
        return hysteresisMinDistance;
      }
      if (distance > hysteresisMaxDistance)
      {
        return hysteresisMaxDistance;
      }
      return distance;
    }
    else
    {
      return hysteresisMinDistance + (hysteresisMaxDistance - hysteresisMinDistance) / 2;
    }
  }

  [[nodiscard]] Angle getOptAngleField(const Vector2f& playerPosition, const Vector2f& ballPosition, const Vector2f& targetPosition) const
  {
    const bool playerOnLeftSideOfKickDirection = Geometry::isPointLeftOfLine(playerPosition, ballPosition, targetPosition);
    const Angle ballToTargetAngle = (targetPosition - ballPosition).angle();
    const Angle optFieldAngleUnnormalized = ballToTargetAngle + (playerOnLeftSideOfKickDirection ? -1.f : 1.f) * optAngle;
    const Angle optFieldAngle = Angle::normalize(optFieldAngleUnnormalized);
    return optFieldAngle;
  }

  [[nodiscard]] virtual Pose2f getKickPose(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, bool leftFootClosestToBall) const = 0;

  virtual void perform(Ballchaser& ballchaser, const Pose2f& kickPose, const Vector2f& targetPosition, bool start) const = 0;

protected:
  const float minDistance;
  const float maxDistance;
  const bool distanceAdjustable;
  const float time;
  const float verticalInaccuracy;
  const float horizontalInaccuracy;
  const float inaccuracy;
  const bool kickBlind;
  const Angle optAngle;
  const float optXDistanceToBall;
  const float optYDistanceToBall;

private:
  const std::string name;
};
