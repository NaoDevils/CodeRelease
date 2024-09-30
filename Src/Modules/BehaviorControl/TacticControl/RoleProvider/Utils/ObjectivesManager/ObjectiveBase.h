#pragma once

#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Utils/Logs/BehaviorLogger.h"

template <class P, class R> class ObjectiveBase
{
public:
  ObjectiveBase(const std::string name) : name(name) {}
  virtual ~ObjectiveBase() = default;

  virtual bool enterCondition() { return true; }
  virtual void preprocess() {}
  virtual bool perform(R& representation) = 0;
  [[nodiscard]] virtual bool leaveCondition() const { return true; }
  virtual void postprocess() {}
  std::string getRawName() { return name; }
  virtual std::string getName() { return name; }

protected:
  const std::string name;
};
