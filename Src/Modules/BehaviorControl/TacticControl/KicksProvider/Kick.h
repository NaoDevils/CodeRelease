#pragma once

#include "Modules/BehaviorControl/TacticControl/RecommendedKickProvider/KickManager/Models/Ranges/DistanceRequirement.h"
#include "Representations/BehaviorControl/RoleSymbols/PositioningAndKickSymbols.h"
#include "Tools/Math/Eigen.h"
#include <utility>

class Kick
{

public:
  Kick(std::string name,
      const float generalValue,
      const float time,
      const float minDistance,
      const float maxDistance,
      const float verticalInaccuracy,
      const Angle horizontalInaccuracy,
      const bool kickBlind,
      const bool switchKickFoot,
      const Angle& optAngle,
      const float optXDistanceToBall,
      const float optYDistanceToBall)
      : name(std::move(name)), generalValue(generalValue), time(time), minDistance(minDistance), maxDistance(maxDistance), verticalInaccuracy(verticalInaccuracy),
        horizontalInaccuracy(horizontalInaccuracy), kickBlind(kickBlind), switchKickFoot(switchKickFoot), optAngle(optAngle), optXDistanceToBall(optXDistanceToBall),
        optYDistanceToBall(optYDistanceToBall)
  {
    ASSERT(time > 0);
    ASSERT(minDistance > 0.f);
    ASSERT(minDistance <= maxDistance);
    ASSERT(minDistance >= horizontalInaccuracy); // otherwise kicks could have a negative distance
    ASSERT(horizontalInaccuracy >= 0.f);
    ASSERT(verticalInaccuracy >= 0_deg);
  }

  virtual ~Kick() = default;

  [[nodiscard]] bool isDistanceAdjustable() const { return minDistance < maxDistance; }

  [[nodiscard]] virtual bool mirrorToKickWithLeft(bool mirror) const
  {
    const bool same = kickWithLeftToMirror(true);
    return same ? mirror : !mirror;
  }

  [[nodiscard]] virtual bool kickWithLeftToMirror(bool kickWithLeft) const;

  [[nodiscard]] virtual bool getKickWithLeftToTurnLeft(bool kickWithLeft) const;

  [[nodiscard]] virtual Pose2f getKickPose(const Vector2f& ballPosition, const Angle targetDirection, bool kickWithLeft) const;

  virtual void perform(PositioningAndKickSymbols& pakSymbols, const Pose2f& kickPose, bool kickPoseMirrored, const Vector2f& targetPosition) const = 0;

  void perform(PositioningAndKickSymbols& pakSymbols, const Vector2f& ballPosition, const Vector2f& targetPosition, bool kickWithLeft) const
  {
    const Angle targetDirection = (targetPosition - ballPosition).angle();
    const Pose2f& kickPose = getKickPose(ballPosition, targetDirection, kickWithLeft);
    bool kickPoseMirrored = kickWithLeftToMirror(kickWithLeft);
    perform(pakSymbols, kickPose, kickPoseMirrored, targetPosition);
  }

  const std::string name;
  const float generalValue;
  const float time;
  const float minDistance;
  const float maxDistance;
  const float verticalInaccuracy;
  const Angle horizontalInaccuracy;
  const bool kickBlind;
  const bool switchKickFoot;
  const Angle optAngle;
  const float optXDistanceToBall;
  const float optYDistanceToBall;
};
