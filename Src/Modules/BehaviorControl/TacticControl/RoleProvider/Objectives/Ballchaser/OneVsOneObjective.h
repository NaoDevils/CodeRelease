#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"

class BallchaserProvider;
struct Ballchaser;
class OneVsOneObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  OneVsOneObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool enterCondition() override;
  void preprocess() override;
  bool perform(Ballchaser& ballchaser) override;
  void postprocess() override;

private:
  KickManager kickManager;
  std::vector<std::unique_ptr<Kick>> kicks;

  bool startTimeSet = false;
  unsigned startTime = 0;
  Vector2f startTimeBallPosition;

  bool playerIsLeftOfBall = false;

  bool kickBallInCone(Ballchaser& ballchaser, Angle toPlayerMaxAngle);

  bool isInBallArea(const Vector2f& ballPosition);

  bool isTimeout();

  bool justKick(Ballchaser& ballchaser);

  void defend(Ballchaser& ballchaser);

  void updateTimer();
};
