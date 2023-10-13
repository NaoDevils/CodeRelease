#pragma once

#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/ObjectiveBase.h"

template <class P, class R> class Objective : public ObjectiveBase<P, R>
{
public:
  Objective(std::string name, const P* role, BehaviorLogger& logger) : ObjectiveBase<P, R>(name), role(role), logger(logger) {}
  virtual ~Objective() = default;

  virtual void preprocess() { isActive = true; }
  virtual void postprocess() { isActive = false; }

protected:
  const P* role;
  bool isActive = false;
  BehaviorLogger& logger;
};
