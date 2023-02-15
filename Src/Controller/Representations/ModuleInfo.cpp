/**
* @file Controller/Representations/ModuleInfo.cpp
*
* Implementation of class ModuleInfo
*
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "ModuleInfo.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include <algorithm>

void ModuleInfo::clear()
{
  modules.clear();
  representations.clear();
  config.representationProviders.clear();
}

void ModuleInfo::clearModules(char processIdentifier)
{
  modules.remove_if(
      [&](const Module& m)
      {
        return m.processIdentifier == processIdentifier;
      });
}

void ModuleInfo::fillRepresentations()
{
  representations.clear();
  for (const Module& module : modules)
  {
    representations.insert(module.requirements.begin(), module.requirements.end());
    representations.insert(module.representations.begin(), module.representations.end());
  }
}


bool ModuleInfo::handleMessage(InMessage& message, char processIdentifier)
{
  if (message.getMessageID() == idModuleTable)
  {
    clearModules(processIdentifier);
    int numOfModules;
    message.bin >> numOfModules;
    for (int i = 0; i < numOfModules; ++i)
    {
      Module module;
      module.processIdentifier = processIdentifier;

      int numOfRequirements;
      unsigned char category;
      message.bin >> module.name >> category >> numOfRequirements;
      module.category = static_cast<ModuleBase::Category>(category);
      module.requirements.resize(numOfRequirements);
      for (unsigned j = 0; j < module.requirements.size(); ++j)
        message.bin >> module.requirements[j];

      int numOfRepresentations;
      message.bin >> numOfRepresentations;
      module.representations.resize(numOfRepresentations);
      for (unsigned j = 0; j < module.representations.size(); ++j)
        message.bin >> module.representations[j];

      std::list<Module>::iterator k;
      for (k = modules.begin(); k != modules.end() && *k < module; ++k)
        ;
      modules.insert(k, std::move(module));
    }
    message.bin >> config;
    timeStamp = SystemCall::getCurrentSystemTime();
    fillRepresentations();
    return true;
  }
  else
    return false;
}

void ModuleInfo::sendRequest(Out& stream, bool sort)
{
  if (sort)
    std::sort(config.representationProviders.begin(), config.representationProviders.end());

  stream << config;
}
