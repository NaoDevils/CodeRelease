#include "BehaviorConfigurationProvider.h"

void BehaviorConfigurationProvider::update(BehaviorConfiguration& behaviorConfiguration)
{
  if (!initialized)
  {
    InMapFile stream("behavior.cfg");
    if (stream.exists())
    {
      stream >> behaviorConfiguration;
    }
    else
      ASSERT(false);
    initialized = true;
  }
}

MAKE_MODULE(BehaviorConfigurationProvider, behaviorControl)