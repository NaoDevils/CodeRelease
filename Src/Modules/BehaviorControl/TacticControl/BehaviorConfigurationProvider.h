/**
* @file BehaviorConfigurationProvider.h
* Module that provides the current behavior configuration(s).
* @author Ingmar Schwarz
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Representations/BehaviorControl/BehaviorConfiguration.h"
MODULE(BehaviorConfigurationProvider,
  PROVIDES(BehaviorConfiguration)
  //PROVIDES(BehaviorConfiguration2015)
);

class BehaviorConfigurationProvider : public BehaviorConfigurationProviderBase
{
public:
  BehaviorConfigurationProvider() { initialized = false; }

private:
  void update(BehaviorConfiguration& behaviorConfiguration);
  bool initialized;
};