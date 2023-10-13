#pragma once

#include <utility>
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/ObjectiveBase.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/ObjectivesManager/ObjectivesManager.h"

template <class P, class R> class ObjectivesList : public ObjectiveBase<P, R>
{
public:
  ObjectivesList(BehaviorLogger* logger) : ObjectiveBase<P, R>("List"), logger(logger) {}

  void add(std::unique_ptr<ObjectiveBase<P, R>> objective) { objectives.push_back(std::move(objective)); }

  void clear()
  {
    objectives.clear();
    currentObjective = nullptr;
  }

  bool enterCondition() override { return true; }

  void preprocess() override {}

  bool perform(R& representation) override
  {
    if (currentObjective == nullptr || currentObjective->leaveCondition())
    {
      findAndPerformObjective(representation);
    }
    else
    {
      bool executed = performCurrentObjective(representation);
      if (!executed)
      {
        findAndPerformObjective(representation);
      }
    }

    if (currentObjective)
    {
      representation.log_currObj = currentObjective->getName();
      return true;
    }
    else
    {
      representation.log_currObj = "None";
      return false;
    }
  }

  [[nodiscard]] bool leaveCondition() const override { return true; }

  void postprocess() override
  {
    if (currentObjective == nullptr)
    {
      return;
    }
    stopObjective(currentObjective);
  }

  std::string getName() override
  {
    if (currentObjective == nullptr)
    {
      return "";
    }
    return currentObjective->getName();
  }

private:
  bool performCurrentObjective(R& positioningSymbols)
  {
    ASSERT(currentObjective);

    logger->start(currentObjective->getName());
    if (currentObjective->perform(positioningSymbols))
    {
      logger->stop();
      return true;
    }
    else
    {
      stopObjective(currentObjective);
      logger->stop();
      return false;
    }
  }

  void findAndPerformObjective(R& positioningSymbols)
  {
    for (const auto& objective : objectives)
    {
      logger->start(objective->getRawName());
      if (!objective->enterCondition())
      {
        logger->stop();
        continue;
      }
      if (objective.get() != currentObjective)
      {
        objective.get()->preprocess();
      }
      if (objective.get()->perform(positioningSymbols))
      {
        logger->stop();
        setCurrentObjective(objective.get());
        return;
      }
      stopObjective(objective.get());
      logger->stop();
    }
  }

  void setCurrentObjective(ObjectiveBase<P, R>* objective)
  {
    if (currentObjective)
    {
      if (currentObjective == objective)
      {
        return;
      }
      stopObjective(currentObjective);
    }
    currentObjective = objective;
  }

  void stopObjective(ObjectiveBase<P, R>* objective)
  {
    objective->postprocess();
    if (currentObjective == objective)
    {
      ANNOTATION("ObjectivesManager", "Stopped currentObjective: " << objective->getName());
      currentObjective = nullptr;
    }
  }

protected:
  ObjectiveBase<P, R>* currentObjective = nullptr;

private:
  std::vector<std::unique_ptr<ObjectiveBase<P, R>>> objectives;
  BehaviorLogger* logger;
};
