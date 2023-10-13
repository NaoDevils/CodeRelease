#pragma once

#include <Tools/Debugging/Annotation.h>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/ObjectivesList.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"

template <class P, class R> class ObjectivesManager
{

public:
  explicit ObjectivesManager(BehaviorLogger* logger) : objectiveList(logger), logger(logger) {}
  ~ObjectivesManager() = default;

  void performObjective(R& positioningSymbols) { objectiveList.perform(positioningSymbols); }

  void add(std::unique_ptr<ObjectiveBase<P, R>> objective) { objectiveList.add(std::move(objective)); }

  void clear() { objectiveList.clear(); }

  void stop() { objectiveList.postprocess(); }

private:
  ObjectivesList<P, R> objectiveList;
  BehaviorLogger* logger;
};
