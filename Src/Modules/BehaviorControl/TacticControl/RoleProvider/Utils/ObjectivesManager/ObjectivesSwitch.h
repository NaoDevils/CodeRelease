#pragma once

#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"
#include "ObjectiveBase.h"
#include "ObjectivesManager.h"

template <class P, class R> class ObjectivesSwitch : public Objective<P, R>
{
public:
  ObjectivesSwitch(BehaviorLogger& logger, const ObjectivesManager<P, R>& objectivesList, const std::function<P>& isSwitchAllowed)
      : ObjectiveBase<P, R>(), objectivesList(std::move(objectivesList)), isSwitchAllowed(std::move(isSwitchAllowed))
  {
  }

  bool enterCondition() override { return objectivesList.enterCondition(); }

  void preprocess() override
  {
    objectivesList.preprocess();
    Objective<P, R>::preprocess();
  }

  bool perform(R& representation) override { return objectivesList.perform(representation); }

  [[nodiscard]] bool leaveCondition() const override
  {
    const bool leave = objectivesList.leaveCondition();
    if (leave)
    {
      return isSwitchAllowed(this->role);
    }
    else
    {
      return false;
    }
  }

  void postprocess() override
  {
    objectivesList.postprocess();
    setSwitchToNeutral();
    Objective<P, R>::postprocess();
  }

private:
  void setSwitchToNeutral() { objectivesList.currentObjective = nullptr; }

  ObjectivesManager<P, R> objectivesList;
  const std::function<P>& isSwitchAllowed;
};
