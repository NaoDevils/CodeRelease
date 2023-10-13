#pragma once

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Enums/KickWithLeftCondition.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Enums/Side.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/DistanceRequirement.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Enums/SelectedFoot.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Tools/Math/Eigen.h"
#include <utility>

class Kick
{
public:
  Kick(std::string name,
      const float minDistance,
      const float maxDistance,
      const bool distanceAdjustable,
      const float time,
      const float horizontalInaccuracy,
      const bool kickBlind,
      KickInfos::KickWithLeftCondition kickWithLeftCondition,
      const bool switchKickFoot,
      const Angle& optAngle,
      const float optXDistanceToBall,
      const float optYDistanceToBall)
      : minDistance(minDistance), maxDistance(maxDistance), distanceAdjustable(distanceAdjustable), time(time), verticalInaccuracy(maxDistance - minDistance),
        horizontalInaccuracy(horizontalInaccuracy), inaccuracy((verticalInaccuracy + horizontalInaccuracy * 1000.f / maxDistance) / 2), kickBlind(kickBlind),
        kickWithLeftCondition(kickWithLeftCondition), switchKickFoot(switchKickFoot), optAngle(optAngle), optXDistanceToBall(optXDistanceToBall),
        optYDistanceToBall(optYDistanceToBall), name(std::move(name))
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

  [[nodiscard]] float getRealisticDistance() const { return (minDistance + maxDistance) / 2; }

  [[nodiscard]] float getRealisticDistance(const bool hysteresis, const bool biggerIsBetter) const
  {
    const float hysteresisMinDistance = getMinDistance(hysteresis, biggerIsBetter);
    const float hysteresisMaxDistance = getMaxDistance(hysteresis, biggerIsBetter);
    return (hysteresisMinDistance + hysteresisMaxDistance) / 2;
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

  [[nodiscard]] virtual bool getKickWithLeft(const Pose2f& playerPose, const Vector2f& ballPosition, const Vector2f& targetPosition, const SelectedFoot currentSelectedFoot) const;

  [[nodiscard]] virtual bool mirrorToKickWithLeft(bool mirror) const
  {
    const bool same = kickWithLeftToMirror(true);
    return same ? mirror : !mirror;
  }

  [[nodiscard]] virtual bool kickWithLeftToMirror(bool kickWithLeft) const;

  [[nodiscard]] virtual bool getKickWithLeftToTurnLeft(bool kickWithLeft) const;

  [[nodiscard]] virtual Pose2f getKickPose(const Vector2f& ballPosition, const Vector2f& targetPosition, bool kickWithLeft) const;

  virtual void perform(PositioningAndKickSymbols& pakSymbols, const Pose2f& kickPose, bool kickPoseMirrored, const Vector2f& targetPosition, bool start) const = 0;

  [[nodiscard]] Angle getOptAngle() const { return optAngle; }

  [[nodiscard]] bool isSwitchKickFoot() const { return switchKickFoot; }

protected:
  const float minDistance;
  const float maxDistance;
  const bool distanceAdjustable;
  const float time;
  const float verticalInaccuracy;
  const float horizontalInaccuracy;
  const float inaccuracy;
  const bool kickBlind;
  const KickInfos::KickWithLeftCondition kickWithLeftCondition;
  const bool switchKickFoot;
  const Angle optAngle;
  const float optXDistanceToBall;
  const float optYDistanceToBall;

private:
  const std::string name;
};
