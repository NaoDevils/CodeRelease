/**
 * @file Module.cpp
 * The class attributes of the module handling schema.
 * @author Thomas Röfer
 */

#include "Module.h"
#include "Tools/Streams/InStreams.h"

ModuleBase* ModuleBase::list = nullptr;

void loadModuleParameters(Streamable& parameters, const char* moduleName, const char* fileName)
{
  std::string name;
  if (!fileName)
    name = getConfigName(moduleName);
  else
    name = fileName;
  InMapFile stream(name);
  if (stream.exists())
    stream >> parameters;
  else
    OUTPUT_WARNING("Parameter file for " << moduleName << " is missing!");
}

std::string getConfigName(const char* moduleName)
{
  std::string name = moduleName;
  name[0] = static_cast<char>(tolower(name[0]));
  if (name.size() > 1 && isupper(name[1]))
    for (int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
      name[i] = static_cast<char>(tolower(name[i]));
  name += ".cfg";
  return name;
}


const ModuleBase::Info* ModuleBase::getInfoByRepresentation(std::string_view representation) const
{
  for (const ModuleBase::Info& i : *infos)
    if (representation == i.representation)
      return &i;
  return nullptr;
}

const ModuleBase::Info* ModuleBase::getInfoByRepresentation(std::string_view representation, ModuleBase::Info::Property property) const
{
  for (const ModuleBase::Info& i : *infos)
    if (representation == i.representation && i.hasProperty(property))
      return &i;
  return nullptr;
}
std::unordered_set<const ModuleBase::Info*> ModuleBase::getInfos(ModuleBase::Info::Property property) const
{
  std::unordered_set<const ModuleBase::Info*> result;
  for (const ModuleBase::Info& it : *infos)
    if (it.hasProperty(property))
      result.insert(&it);
  return result;
}
