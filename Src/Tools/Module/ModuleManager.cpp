/**
 * @file ModuleManager.cpp
 * Implementation of a class representing the module manager.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "ModuleManager.h"
#include "Platform/BHAssert.h"
#include <algorithm>
#include "Platform/File.h"
#include <unordered_set>

#include "Tools/ProcessFramework/SubThread.h"
#include <taskflow/taskflow.hpp>

ModuleManager::Configuration::RepresentationProvider::RepresentationProvider(std::string representation, std::string provider)
    : representation(std::move(representation)), provider(std::move(provider))
{
}

CycleLocal<ModuleManager*> ModuleManager::theInstance{nullptr};

ModuleManager::ModuleManager(const std::set<ModuleBase::Category>& categories, SuperThread* superthread) : taskflow(std::make_unique<tf::Taskflow>())
{
  this->superthread = superthread;

  for (ModuleBase& i : *ModuleBase::list)
    if (categories.find(i.category) != categories.end())
      modules.emplace_back(&i);
    else
      otherModules.emplace_back(&i);

  theInstance = this;
}

ModuleManager::~ModuleManager()
{
  destroy();
  theInstance.reset();
}

void ModuleManager::destroy()
{
  if (!providers.empty())
  {
    char buf[100];
    OutBinaryMemory out(buf);
    out << Configuration();
    InBinaryMemory in(buf);
    update(in, 0); // destruct everything
  }
}

std::optional<std::tuple<std::unordered_set<const char*>, std::unordered_set<const char*>>> ModuleManager::calcShared(const Configuration& config) const
{
  std::unordered_set<const char*> received, sent;

  for (const auto& representationProvider : config.representationProviders)
  {
    auto module = std::find(modules.begin(), modules.end(), representationProvider.provider);
    auto otherModule = std::find(otherModules.begin(), otherModules.end(), representationProvider.provider);
    if (module == modules.end() && otherModule == otherModules.end())
    {
      // default can provide everthing that exists, but only that
      if (representationProvider.provider == "default")
      {
        const auto hasRepresentation = [&](const auto& m)
        {
          return m.getInfoByRepresentation(representationProvider.representation) != nullptr;
        };

        if (!std::any_of(ModuleBase::list->begin(), ModuleBase::list->end(), hasRepresentation))
        {
          OUTPUT_ERROR("Unknown representation " << representationProvider.representation << "!");
          return {};
        }
      }
      else
      {
        OUTPUT_ERROR("Module " << representationProvider.provider << " is unknown!");
        return {};
      }
    }
    else
    {
      bool provided = false;
      if (module != modules.end())
      {
        // check if module really provides this representation
        provided |= module->module->getInfoByRepresentation(representationProvider.representation, Property::provide) != nullptr;
        if (!calcShared(config, representationProvider.representation, *module, modules, received, false))
          return {};
      }
      if (otherModule != otherModules.end())
      {
        provided |= otherModule->module->getInfoByRepresentation(representationProvider.representation, Property::provide) != nullptr;
        if (!calcShared(config, representationProvider.representation, *otherModule, otherModules, sent, true))
          return {};
      }
      if (!provided)
      {
        OUTPUT_ERROR(representationProvider.provider << " does not provide " << representationProvider.representation << "!");
        return {};
      }
    }
  }
  return {{std::move(received), std::move(sent)}};
}

bool ModuleManager::calcShared(
    const Configuration& config, std::string_view representation, const ModuleState& module, const std::list<ModuleState>& modules, std::unordered_set<const char*>& received, bool silent)
{
  const auto isProvidedHere = [&](std::string_view provider)
  {
    return provider == "default" || std::find(modules.begin(), modules.end(), provider) != modules.end();
  };

  for (const std::string& providerOfRepresentation : config.getProvidersByRepresentation(representation))
  {
    // check multiple providers for same representation
    if (providerOfRepresentation != module.module->name && isProvidedHere(providerOfRepresentation))
    {
      if (!silent)
        OUTPUT_ERROR("Representation " << std::string(representation) << " provided by more than one module!");
      return false;
    }
  }

  const auto representationProvidedHere = [&](std::string_view representation)
  {
    return [&, representation](const std::string& provider)
    {
      const auto module = std::find(modules.begin(), modules.end(), provider);
      return provider == "default" || (module != modules.end() && module->module->getInfoByRepresentation(representation, Property::provide) != nullptr);
    };
  };

  for (const ModuleBase::Info* requirement : module.module->getInfos(Property::require))
  {
    const auto configuredProviders = config.getProvidersByRepresentation(requirement->representation);

    bool providedHere = std::any_of(configuredProviders.begin(), configuredProviders.end(), representationProvidedHere(requirement->representation));

    if (configuredProviders.empty())
    {
      if (!silent)
        OUTPUT_ERROR("No provider for representation " << requirement->representation << " required by " << module.module->name << "!");
      return false;
    }
    else if (!providedHere)
      received.insert(requirement->representation);
  }

  for (const ModuleBase::Info* requirement : module.module->getInfos(Property::use))
  {
    const auto configuredProviders = config.getProvidersByRepresentation(requirement->representation);

    bool providedHere = std::any_of(configuredProviders.begin(), configuredProviders.end(), representationProvidedHere(requirement->representation));

    if (configuredProviders.empty())
    {
      if (!silent)
        OUTPUT_ERROR("No provider for representation " << requirement->representation << " used by " << module.module->name << "!");
      return false;
    }
    else if (!providedHere)
    {
      received.insert(requirement->representation);
      //OUTPUT_ERROR("Cannot use representation " << requirement->representation << " by " << module.module->name << " from different cycle!");
      //return false;
    }
  }
  return true;
}

ModuleManager::Configuration ModuleManager::mergeConfig(ModuleManager::Configuration& config, In& stream)
{
  ModuleManager::Configuration newConfig;
  stream >> newConfig;

  return mergeConfig(config, newConfig);
}

ModuleManager::Configuration ModuleManager::mergeConfig(ModuleManager::Configuration& config, const ModuleManager::Configuration& newConfig)
{
  ModuleManager::Configuration oldConfig;

  auto& oldProviders = oldConfig.representationProviders;
  auto& currentProviders = config.representationProviders;
  auto& newProviders = newConfig.representationProviders;

  const auto hasSameRepresentation = [&](const auto& currentProvider)
  {
    return std::any_of(newProviders.begin(),
        newProviders.end(),
        [&](const auto& newProvider)
        {
          return currentProvider.representation == newProvider.representation;
        });
  };

  const auto part = std::stable_partition(currentProviders.begin(), currentProviders.end(), std::not_fn(hasSameRepresentation));

  oldProviders = {std::make_move_iterator(part), std::make_move_iterator(currentProviders.end())};

  currentProviders.erase(part, currentProviders.end());

  currentProviders.insert(currentProviders.end(), newProviders.begin(), newProviders.end());

  return oldConfig;
}

void ModuleManager::update(In& stream, unsigned timeStamp)
{
  stream >> config;

  if (updateProviders())
  {
    this->nextTimeStamp = timeStamp; // use this timestamp after execute was called
    this->timeStamp = 0; // invalid until execute was called
  }
}

bool ModuleManager::updateProviders()
{
  // fill shared representations
  auto shared = calcShared(config);
  if (!shared.has_value())
    return false;

  auto& [received, sent] = shared.value();
  std::list<Provider> providers;

  // remove all markings
  for (auto& m : modules)
  {
    m.required = false;
  }

  // fill providers list
  for (const auto& rp : config.representationProviders)
  {
    const auto m = std::find(modules.begin(), modules.end(), rp.provider);
    if (const ModuleBase::Info * info; m != modules.end() && (info = m->module->getInfoByRepresentation(rp.representation, Property::provide)))
    {
      providers.emplace_back(info->representation, &*m, info);
      m->required = true;
    }
  }

  // generate task graph
  std::unique_ptr<tf::Taskflow> taskflow = generateTaskflow(providers);

  // check cycles
  std::list<std::string> cycle = findCyclicDependencies(*taskflow);
  if (cycle.size() > 0)
  {
    const auto join = [](const std::string& a, const std::string& b)
    {
      return a + (a.length() > 0 ? " => " : "") + b;
    };

    const std::string cycleText = std::accumulate(cycle.begin(), cycle.end(), std::string(), join);
    OUTPUT_ERROR("Cyclic dependencies detected: " << cycleText);
    return false;
  }

  // apply new configuration
  this->providers = std::move(providers);
  this->taskflow = std::move(taskflow);
  this->received = std::move(received);
  this->sent = std::move(sent);

  // delete all modules that are not required anymore
  // create new modules that are required
  for (auto& m : modules)
  {
    if (!m.required && m.instance)
    {
      m.instance.reset();
    }
    else if (m.required && !m.instance)
    {
      m.instance = m.module->createNew();
    }
  }

  return true;
}


std::unique_ptr<tf::Taskflow> ModuleManager::generateTaskflow(const std::list<Provider>& providers)
{
  std::unique_ptr<tf::Taskflow> taskflow = std::make_unique<tf::Taskflow>();

  // remember tasks
  std::map<const Provider*, tf::Task> updateTasks;
  std::map<const ModuleBase*, tf::Task> executeTasks;

  std::map<const ModuleBase*, std::tuple<std::list<tf::Task*>, std::list<tf::Task*>>> tasksOfModules;

  // create tasks
  for (const Provider& provider : providers)
  {
    // add update methods
    updateTasks[&provider] =
        taskflow
            ->emplace(
                [&]()
                {
                  if (provider.moduleState->instance)
                    provider.update(*provider.moduleState->instance);
                })
            .name(std::string(provider.representation) + " [" + provider.moduleState->module->name + "]");

    // remember tasks for update methods of modules to add dependencies later
    auto& [sequentialTasks, concurrentTasks] = tasksOfModules[provider.moduleState->module];
    if (provider.info->hasProperty(Property::concurrent))
      concurrentTasks.push_back(&updateTasks[&provider]);
    else
      sequentialTasks.push_back(&updateTasks[&provider]);

    // add pre-execution methods
    for (const ModuleBase::Info* info : provider.moduleState->module->getInfos(Property::preexecution))
    {
      if (executeTasks.find(provider.moduleState->module) == executeTasks.end())
      {
        executeTasks[provider.moduleState->module] =
            taskflow
                ->emplace(
                    [&, execute = info->execute](tf::Subflow& subflow)
                    {
                      if (provider.moduleState->instance)
                        execute(*provider.moduleState->instance, subflow);
                    })
                .name(std::string(provider.moduleState->module->name) + " [" + provider.moduleState->module->name + "]");
      }
    }
  }

  // a module is able to provide and require the same representation
  // in this case, the corresponding update method is guaranteed to be executed first
  // remember this first update method here
  std::map<const ModuleBase*, tf::Task*> firstUpdateOfModules;

  // add dependencies of update methods
  for (auto& [provider1, task] : updateTasks)
  {
    for (const ModuleBase::Info* requirement : provider1->moduleState->module->getInfos(Property::require))
    {
      auto provider2 = std::find(providers.begin(), providers.end(), requirement->representation);

      // skip requirements provided by default
      if (provider2 == providers.end())
        continue;

      // skip same representation
      if (provider1 == &*provider2)
        continue;

      // skip module that depends on itself
      if (provider1->moduleState->module == provider2->moduleState->module)
      {
        // remember first update method
        const auto firstUpdate = firstUpdateOfModules.find(provider2->moduleState->module);
        if (firstUpdate == firstUpdateOfModules.end())
        {
          firstUpdateOfModules[provider2->moduleState->module] = &updateTasks[&*provider2];
          continue;
        }
        // skip if we have already remebered the same update method
        else if (firstUpdate->second == &updateTasks[&*provider2])
          continue;
      }

      updateTasks[&*provider2].precede(task);
    }
  }

  // add dependencies of pre-executions
  for (auto& [executeModule, executeTask] : executeTasks)
  {
    for (const ModuleBase::Info* requirement : executeModule->getInfos(ModuleBase::Info::Property::require))
    {
      auto provider = std::find(providers.begin(), providers.end(), requirement->representation);

      // skip requirements provided by default
      if (provider == providers.end())
        continue;

      // skip representations that are provided and required by same module
      if (executeModule == provider->moduleState->module)
        continue;

      updateTasks[&*provider].precede(executeTask);
    }
  }

  // precalculate successor counts
  std::unordered_map<const tf::Task*, size_t> numSuccessors;
  for (const auto& [_, task] : updateTasks)
  {
    std::unordered_set<size_t> visited;
    const std::function<void(tf::Task)> addToVisited = [&](tf::Task t)
    {
      if (visited.find(t.hash_value()) == visited.end())
      {
        visited.insert(t.hash_value());
        t.for_each_successor(addToVisited);
      }
    };
    task.for_each_successor(addToVisited);

    numSuccessors[&task] = visited.size();
  }

  // add dependencies between update/pre-execution methods of the same module
  for (auto& [module, tasks] : tasksOfModules)
  {
    auto& [sequentialTasks, concurrentTasks] = tasks;

    const auto firstUpdate = firstUpdateOfModules.find(module);
    const auto decreasingSuccessors = [&](const tf::Task* t1, const tf::Task* t2)
    {
      // prefer first update method
      if (firstUpdate != firstUpdateOfModules.end())
      {
        if (firstUpdate->second == t1)
          return true;
        else if (firstUpdate->second == t2)
          return false;
      }

      return numSuccessors[t1] > numSuccessors[t2];
    };
    sequentialTasks.sort(decreasingSuccessors);

    // 1. run pre-execution
    tf::Task* prevTask = executeTasks.find(module) != executeTasks.end() ? &executeTasks[module] : nullptr;

    // 2. run concurrent updates
    for (tf::Task* concurrentTask : concurrentTasks)
    {
      if (prevTask)
        prevTask->precede(*concurrentTask);

      if (sequentialTasks.size() > 0)
        concurrentTask->precede(*sequentialTasks.front());

      if (concurrentTask == concurrentTasks.back())
        prevTask = nullptr;
    }

    // 3. run sequential updates
    for (tf::Task* task : sequentialTasks)
    {
      if (prevTask)
        prevTask->precede(*task);
      prevTask = task;
    }
  }

  return taskflow;
}

std::list<std::string> ModuleManager::findCyclicDependencies(const tf::Taskflow& tf)
{
  // transform Taskflow to simple graph structure for easy traversal
  using Vertex = std::string;
  using Graph = std::unordered_map<Vertex, std::unordered_set<Vertex>>;
  Graph graph;

  tf.for_each_task(
      [&](const tf::Task& task)
      {
        auto& pre = graph[task.name()];
        task.for_each_successor(
            [&](const tf::Task& suc)
            {
              pre.insert(suc.name());
            });
      });

  // using depth first search for cycle detection
  // see: https://de.wikipedia.org/wiki/Zyklus_(Graphentheorie)
  const auto checkCycle = [](const Graph& graph, const Vertex& v)
  {
    std::unordered_set<Vertex> visited, finished;
    using DepthFirstSearch = std::function<std::list<Vertex>(const Vertex&)>;

    const DepthFirstSearch dfs = [&](const Vertex& v) -> std::list<Vertex>
    {
      if (finished.count(v))
        return {};
      if (visited.count(v))
        return {v};
      visited.insert(v);
      for (const Vertex& suc : graph.at(v))
      {
        std::list<Vertex> ret = dfs(suc);
        if (ret.size() > 0)
        {
          if (ret.size() < 2 || ret.front() != ret.back())
            ret.push_front(v);
          return ret;
        }
      }
      finished.insert(v);
      return {};
    };
    return dfs(v);
  };

  std::list<Vertex> result;
  tf.for_each_task(
      [&](const tf::Task& task)
      {
        if (result.size() == 0)
          result = checkCycle(graph, task.name());
      });
  return result;
}

void ModuleManager::load()
{
  for (const std::string& name : File::getFullNamesHierarchy("modules.cfg"))
  {
    InMapFile stream(name);
    if (!stream.exists())
    {
      OUTPUT_ERROR("failed to load modules.cfg correctly.");
      ASSERT(true); // since when modules aren't loaded correctly there come up other failures
    }
    mergeConfig(config, stream);
  }

  if (updateProviders())
  {
    // we cannot use system time here because we need the same value in motion and cognition
    this->nextTimeStamp = 1;
    this->timeStamp = 0;
  }
}

void ModuleManager::execute()
{
  this->superthread->run(*taskflow);

  if (!timeStamp) // Configuration changed recently?
  { // all representations must be constructed now, so we can receive data
    timeStamp = nextTimeStamp;
    toSend.clear();
    for (const auto& s : sent)
      toSend.push_back(&Blackboard::getInstance()[s]);
    toReceive.clear();
    for (const auto& r : received)
      toReceive.push_back(&Blackboard::getInstance()[r]);
  }

  const ModuleManager::NextConfig* nextModuleConfig = nextConfig.load(std::memory_order_acquire);
  if (nextModuleConfig)
  {
    if (nextModuleConfig->timestamp > nextTimeStamp)
    {
      config = nextModuleConfig->config;

      sendModuleTable = true;
      writeModuleConfig = true;

      if (updateProviders())
      {
        this->nextTimeStamp = nextModuleConfig->timestamp;
        this->timeStamp = 0;
      }
    }

    delete nextModuleConfig;
    nextConfig.store(nullptr, std::memory_order_relaxed);
  }

  DEBUG_RESPONSE_ONCE("automated requests:ModuleTable")
  sendModuleTable = true;

  if (sendModuleTable)
  {
    sendModuleTable = false;

    Global::getDebugOut().bin << static_cast<unsigned>(modules.size());
    for (auto& m : modules)
    {
      Global::getDebugOut().bin << m.module->name << static_cast<unsigned char>(m.module->category);

      const auto require = m.module->getInfos(Property::require);
      Global::getDebugOut().bin << static_cast<unsigned>(require.size());
      for (const ModuleBase::Info* j : require)
        Global::getDebugOut().bin << j->representation;

      const auto provide = m.module->getInfos(Property::provide);
      Global::getDebugOut().bin << static_cast<unsigned>(provide.size());
      for (const ModuleBase::Info* j : provide)
        Global::getDebugOut().bin << j->representation;
    }

    Global::getDebugOut().bin << config;
    Global::getDebugOut().finishMessage(idModuleTable);
  }
}

void ModuleManager::readPackage(In& stream)
{
  unsigned timeStamp;
  bool readModuleConfig;
  stream >> timeStamp >> readModuleConfig;

  // Communication is only possible if both sides are based on the same module request.
  if (timeStamp && this->timeStamp && timeStamp == this->timeStamp)
  {
    // throw away received config
    if (readModuleConfig)
    {
      ModuleManager::Configuration dummy;
      unsigned nextTimeStamp;
      stream >> nextTimeStamp >> dummy;
    }

    writeModuleConfig = false;
    for (Streamable* s : toReceive)
      stream >> *s;
  }
  else
  {
    if (readModuleConfig)
    {
      unsigned nextTimeStamp;
      stream >> nextTimeStamp;

      if (nextTimeStamp > this->nextTimeStamp)
      {
        stream >> config;
        sendModuleTable = true;
        // we should not keep sending our own config when we receive one
        writeModuleConfig = false;
        if (updateProviders())
        {
          this->nextTimeStamp = nextTimeStamp;
          this->timeStamp = 0;
        }
      }
    }

    stream.skip(10000000); // skip everything
  }
}

void ModuleManager::writePackage(Out& stream) const
{
  stream << timeStamp << writeModuleConfig;

  if (writeModuleConfig)
    stream << nextTimeStamp << config;


  for (const Streamable* s : toSend)
    stream << *s;
}

ModuleManager::Configuration ModuleManager::sendModuleRequest(const ModuleManager::Configuration& configRequest)
{
  // With C++17, smart pointers cannot be used with std::atomic :-(
  ModuleManager::Configuration config((*theInstance)->config);
  const ModuleManager::Configuration oldConfig = mergeConfig(config, configRequest);

  const ModuleManager::NextConfig* expected = nullptr;
  const ModuleManager::NextConfig* nextConfig = new ModuleManager::NextConfig{SystemCall::getCurrentSystemTime(), std::move(config)};

  // Implement error handling / rollback if another module request is still being processed?
  VERIFY((*theInstance)->nextConfig.compare_exchange_strong(expected, nextConfig, std::memory_order_release, std::memory_order_relaxed));

  return oldConfig;
}
