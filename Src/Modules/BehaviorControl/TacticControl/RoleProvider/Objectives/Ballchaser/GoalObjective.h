#pragma once

#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/FastLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/SlowLongKick.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/Kicks/KickHack.h>
#include <Modules/BehaviorControl/TacticControl/RoleProvider/Kick/KickManager.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/BallchaserProvider.h"
#include "Representations/BehaviorControl/RoleSymbols/Ballchaser.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/FieldUtils.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/KickUtils.h"

class BallchaserProvider;
struct Ballchaser;
class GoalObjective : public Objective<BallchaserProvider, Ballchaser>
{

public:
  GoalObjective(BallchaserProvider* role, BehaviorLogger& logger);
  bool enterCondition() override;
  bool perform(Ballchaser& ballchaser) override;

  enum GoalDistance
  {
    VERY_FAR,
    FAR,
    NORMAL,
    CLOSE,
    VERY_CLOSE
  };

private:
  KickManager kickManager;
  std::vector<std::unique_ptr<Kick>> kicks = {};
  bool openingAngleTooSharp = false;
  GoalDistance goalDistance = FAR;

  void update();

  bool isVeryCloseToGoal(const Vector2f& leftGoalPostPosition, const Vector2f& rightGoalPostPosition, float ballToGoalDistance);

  bool kickToGoalCenter(Ballchaser& ballchaser);

  bool kickBestToGoal(Ballchaser& ballchaser);
};
