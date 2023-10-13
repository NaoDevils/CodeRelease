#pragma once

#include <Modules/BehaviorControl/TacticControl/RoleProvider/KickManager/Models/KickManager.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Ballchaser/BallchaserProvider.h"
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
  void postprocess() override;

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
  std::vector<std::unique_ptr<Kick>> dangerKicks = {};
  std::vector<std::unique_ptr<Kick>> noDangerKicks = {};
  bool openingAngleTooSharp = false;
  GoalDistance goalDistance = FAR;

  void updateGoalDistance();
  bool isVeryCloseToGoal() const;

  bool kickToGoalCenter(Ballchaser& ballchaser);

  bool kickBestToGoal(Ballchaser& ballchaser);
};
