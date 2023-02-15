/**
 * @file CycleLocal.h
 *
 * Contains the definition of class CycleLocal.
 */

#pragma once

#include "Tools/Module/Blackboard.h"
#include "Platform/BHAssert.h"
#include "Tools/Build.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include <functional>
#include <atomic>


template <typename V> class CycleLocal
{
private:
  std::function<std::unique_ptr<V>()> init;

  static constexpr size_t maxNumberOfRobots = Build::targetRobot() ? 1 : MAX_NUM_PLAYERS * 2;
  static constexpr size_t maxNumberOfCycles = maxNumberOfRobots * 3; // # robots * 2 teams * 3 cycles
  std::atomic_uchar maxCycleId{0};
  std::array<std::unique_ptr<V>, maxNumberOfCycles> entries{nullptr};
  std::array<const Blackboard*, maxNumberOfCycles> cycleIds{nullptr};
  std::array<std::atomic<bool>, maxNumberOfCycles> freeEntries{false};

  inline static thread_local size_t cache = 0;

public:
  template <typename... Params>
  CycleLocal(Params&&... params)
      : init(
          [=]
          {
            return std::make_unique<V>(params...);
          })
  {
  }
  CycleLocal(std::function<std::unique_ptr<V>()>&& init) : init(std::forward<std::function<std::unique_ptr<V>()>>(init)) {}

  V& operator*()
  {
    const Blackboard* const instance = &Blackboard::getInstance();
    ASSERT(instance);

    if (cycleIds[cache] == instance)
      return *entries[cache];

    for (unsigned char id = 0; id < maxCycleId.load(std::memory_order_relaxed); ++id)
    {
      if (cycleIds[id] == instance)
      {
        cache = id;
        return *entries[cache];
      }
    }

    unsigned char newId = maxCycleId.fetch_add(1, std::memory_order_relaxed);
    if (newId >= maxNumberOfCycles)
    {
      maxCycleId.store(maxNumberOfCycles, std::memory_order_relaxed);
      for (newId = 0; newId < maxNumberOfCycles; ++newId)
      {
        if (freeEntries[newId].exchange(false, std::memory_order_relaxed))
          break;
      }
    }
    ASSERT(newId < maxNumberOfCycles);

    entries[newId] = init();
    cycleIds[newId] = instance;

    cache = newId;
    return *entries[cache];
  }


  V* operator->() { return &**this; }
  operator V&() { return **this; }
  V& operator=(V&& elem) { return (**this = std::forward<V>(elem)); }

  void reset()
  {
    const Blackboard* const instance = &Blackboard::getInstance();
    ASSERT(instance);

    for (unsigned char id = 0; id < maxCycleId.load(std::memory_order_relaxed); ++id)
    {
      if (cycleIds[id] == instance)
      {
        entries[id].reset();
        cycleIds[id] = nullptr;
        freeEntries[id].store(true, std::memory_order_relaxed);
        return;
      }
    }
    ASSERT(false);
  }
};
