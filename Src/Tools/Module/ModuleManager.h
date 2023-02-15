/**
 * @file ModuleManager.h
 * Declaration of a class representing the module manager.
 * @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Module.h"
#include "Tools/Streams/AutoStreamable.h"
#include <list>
#include <map>
#include <set>
#include <vector>
#include "Tools/ProcessFramework/CycleLocal.h"
#include <memory>
#include <unordered_set>
#include <string_view>
#include <atomic>
#include <initializer_list>
#include <optional>

class SuperThread;
namespace tf
{
  class Taskflow;
}


/**
 * @class ModuleManager
 * A class representing the module manager.
 */
class ModuleManager
{
private:
  using Property = ModuleBase::Info::Property;

  /**
   * The class represents the current state of a module.
   */
  class ModuleState
  {
  public:
    ModuleBase* module; /**< A pointer to the module base that is able to create an instance of the module. */
    std::unique_ptr<Streamable> instance = nullptr; /**< A pointer to the instance of the module if it was created. Otherwise the pointer is 0. */
    bool required = false; /**< A flag that is required when determining whether a module is currently required or not. */

    /**
     * Constructor.
     * @param module A pointer to the module base that is able to create an instance of the module.
     */
    ModuleState(ModuleBase* module) : module(module) {}

    /**
     * Comparison operator. Only uses the name for comparison.
     * @param name The module name this one is compared to.
     * @return Is this the name of the module?
     */
    bool operator==(std::string_view name) const { return module->name == name; }
  };

  /**
   * The class represents a provider of information, i.e. a method of a module that updates a representation.
   */
  class Provider
  {
  public:
    const char* representation; /**< The representation that will be provided. */
    ModuleState* moduleState; /**< The moduleState that will give access to the module that provides the information. */
    const ModuleBase::Info* info; /**< The module info that will give access to the properties. */
    void (*update)(Streamable&); /**< The update handler within the module. */

    /**
     * Constructor.
     * @param representation The name of the representation provided.
     * @param moduleState The moduleState that will give access to the module that provides the information.
     * @param update The update handler within the module.
     */
    Provider(const char* representation, ModuleState* moduleState, const ModuleBase::Info* info)
        : representation(representation), moduleState(moduleState), info(info), update(info->update)
    {
    }

    operator std::string() const { return representation; }

    bool operator==(const char* name) const { return representation == name; }
  };

public:
  /**
   * The class is used for reading configurations.
   *
   * This class is public since it is used for the ModuleInfo also.
   */
  STREAMABLE(Configuration,
    STREAMABLE(RepresentationProvider,
      RepresentationProvider() = default;
      RepresentationProvider(std::string representation, std::string provider);

      /**
        * Comparison operator. Uses the representation name for comparison.
        * @param other The other RepresentationProvider this one is compared to.
        * @return Is this representations name lower than the other?
        */
      bool operator<(const RepresentationProvider & other) const
      {
        return (representation < other.representation);
      },

      (std::string) representation,
      (std::string) provider
    );

    Configuration() = default;
    Configuration(std::initializer_list<RepresentationProvider> providers) :
      representationProviders(providers) {}
    
    std::unordered_set<std::string> getProvidersByRepresentation(std::string_view representation) const
    {
      std::unordered_set<std::string> result;
      for (const auto& repPro : representationProviders)
      {
        if (repPro.representation == representation)
          result.insert(repPro.provider);
      }
      return result;
    },

    (std::vector<RepresentationProvider>) representationProviders
  );

private:
  Configuration config; /**< The last configuration set. It may not work. */
  std::list<Provider> providers; /**< The list of providers that will be executed. */
  std::list<ModuleState> modules; /**< The current state of all modules. */
  std::list<ModuleState> otherModules; /**< The modules in other processes. */
  std::unordered_set<const char*> sent; /**< The list of all names of representations sent to the other process */
  std::unordered_set<const char*> received; /**< The list of all names of representations received from the other process */
  std::vector<Streamable*> toSend; /**< The list of all representations sent to the other process */
  std::vector<Streamable*> toReceive; /**< The list of all representations received from the other process */
  unsigned timeStamp = 0; /**< The timestamp of the last module request. Communication is only possible if both sides use the same timestamp. */
  unsigned nextTimeStamp = 0; /**< The next timestamp used to verify communication. */

  SuperThread* superthread;
  std::unique_ptr<tf::Taskflow> taskflow;

  struct NextConfig
  {
    unsigned timestamp;
    ModuleManager::Configuration config;
  };
  static CycleLocal<ModuleManager*> theInstance;
  std::atomic<const ModuleManager::NextConfig*> nextConfig{nullptr};
  bool sendModuleTable = false;
  bool writeModuleConfig = false;

public:
  /**
   * Constructor used when framework processes are mapped to threads.
   * In that case, each process sees all modules, because they are all declared in the
   * same address space. Therefore, this constructor filters them based on their categories.
   * @param categories The categories of modules executed by this process.
   */
  ModuleManager(const std::set<ModuleBase::Category>& categories, SuperThread* process);

  /**
   * Destructor.
   * Destructs all modules currently constructed.
   */
  ~ModuleManager();

  /**
   * The method loads the selection of solutions from a configuration file.
   */
  void load();

  /**
   * The method destroys all modules. It can be called to destroy the modules
   * before the constructor is called.
   */
  void destroy();

  /**
   * The method updates the list of the currently created modules.
   * @param stream The stream the new configuration is read from.
   * @param timeStamp The timeStamp of the last module request.
   */
  void update(In& stream, unsigned timeStamp);

  /**
   * Update dependencies.
   */
  bool updateProviders();

  /**
   * The method executes all selected modules.
   */
  void execute();

  /**
   * The method reads a package from a stream.
   * @param stream A stream containing representations received from another process.
   */
  void readPackage(In& stream);

  /**
   * The method writes a package to a stream.
   * @param stream A stream that will be filled with representations that are sent
   *               to another process.
   */
  void writePackage(Out& stream) const;

  static ModuleManager::Configuration sendModuleRequest(const ModuleManager::Configuration& configRequest);

private:
  /**
   * Adds all representations that need to be shared between processes to the
   * attributes "sent" and "received".
   * @param config The module configuration that currently set up.
   * @return Everything ok? If not, the new configuration is invalid.
   */
  std::optional<std::tuple<std::unordered_set<const char*>, std::unordered_set<const char*>>> calcShared(const Configuration& config) const;

  /**
   * Adds all representations that need to be received by a process to the
   * parameter "received". This version is called for each configuration entry.
   * @param config The module configuration that currently set up.
   * @param representation The representation that should be provided.
   * @param module The module that should provide the representation.
   * @param modules All modules in the same process as "module".
   * @param received The list of representations to update.
   * @param silent Suppress error output, because the other process will output them.
   * @return Is the representation really provided by this module?
   */
  static bool calcShared(
      const Configuration& config, std::string_view representation, const ModuleState& module, const std::list<ModuleState>& modules, std::unordered_set<const char*>& received, bool silent);

  static std::list<std::string> findCyclicDependencies(const tf::Taskflow& tf);

  static std::unique_ptr<tf::Taskflow> generateTaskflow(const std::list<Provider>& providers);

  static ModuleManager::Configuration mergeConfig(ModuleManager::Configuration& config, const ModuleManager::Configuration& newConfig);
  static ModuleManager::Configuration mergeConfig(ModuleManager::Configuration& config, In& stream);
};
