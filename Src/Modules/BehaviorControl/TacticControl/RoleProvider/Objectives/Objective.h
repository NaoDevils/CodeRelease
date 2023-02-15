#pragma once

#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"

template <class P, class R> class Objective
{
public:
  Objective(std::string name, const P* role, BehaviorLogger& logger) : role(role), logger(logger), name(std::move(name)) {}
  virtual ~Objective() = default;

  virtual bool enterCondition() { return true; }
  virtual void preprocess() { isActive = true; }
  virtual bool perform(R& representation) = 0;
  [[nodiscard]] virtual bool leaveCondition() const { return true; }
  virtual void postprocess() { isActive = false; }
  std::string getName() { return name; }

protected:
  const P* role;
  bool isActive = false;
  BehaviorLogger& logger;

private:
  const std::string name;
};
