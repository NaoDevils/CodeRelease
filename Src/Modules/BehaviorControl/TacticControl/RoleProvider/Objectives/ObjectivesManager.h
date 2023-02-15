#pragma once

#include "Modules/BehaviorControl/TacticControl/RoleProvider/Objectives/Objective.h"
#include "Modules/BehaviorControl/TacticControl/RoleProvider/Logs/BehaviorLogger.h"

template <class PR, class PS> class ObjectivesManager
{

public:
  explicit ObjectivesManager(BehaviorLogger& logger) : logger(logger) {}
  ~ObjectivesManager() = default;

  void executeObjective(PS& positioningSymbols)
  {
    bool executed = executeCurrentObjective(positioningSymbols);

    if (!executed)
    {
      findAndExecuteNewObjective(positioningSymbols);
    }

    if (currentObjective)
    {
      positioningSymbols.log_currObj = currentObjective->getName();
    }
    else
    {
      positioningSymbols.log_currObj = "None";
    }
  }

  /**
   * @brief add most important objective first
  */
  void add(std::unique_ptr<Objective<PR, PS>> objective) { objectives.push_back(std::move(objective)); }

private:
  bool executeCurrentObjective(PS& positioningSymbols)
  {
    if (currentObjective)
    {
      if (currentObjective->leaveCondition())
      {
        return false;
      }
      else
      {
        logger.start(currentObjective->getName());
        if (currentObjective->perform(positioningSymbols))
        {
          logger.stop();
          return true;
        }
        else
        {
          stopObjective(currentObjective);
          logger.stop();
          return false;
        }
      }
    }
    return false;
  }

  void findAndExecuteNewObjective(PS& positioningSymbols)
  {
    for (const auto& objective : objectives)
    {
      logger.start(objective->getName());
      if (objective->enterCondition())
      {
        if (objective.get() != currentObjective)
        {
          objective.get()->preprocess();
        }
        if (objective.get()->perform(positioningSymbols))
        {
          setCurrentObjective(objective.get());
          logger.stop();
          return;
        }
        else
        {
          stopObjective(objective.get());
        }
      }
      logger.stop();
    }
  }

  void setCurrentObjective(Objective<PR, PS>* objective)
  {
    if (currentObjective)
    {
      if (currentObjective == objective)
      {
        return;
      }
      currentObjective->postprocess();
    }
    currentObjective = objective;
  }

  void stopObjective(Objective<PR, PS>* objective)
  {
    objective->postprocess();
    if (currentObjective == objective)
    {
      currentObjective = nullptr;
    }
  }

  Objective<PR, PS>* currentObjective = nullptr;
  std::vector<std::unique_ptr<Objective<PR, PS>>> objectives;
  BehaviorLogger& logger;
};
