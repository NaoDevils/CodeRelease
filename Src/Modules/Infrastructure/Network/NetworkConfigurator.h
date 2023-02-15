#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/NetworkStatus.h"
#include "Representations/Infrastructure/USBSettings.h"
#include <future>

MODULE(NetworkConfigurator,
  REQUIRES(USBSettings),
  PROVIDES(NetworkStatus)
);

class NetworkConfigurator : public NetworkConfiguratorBase
{
private:
  void update(NetworkStatus& usbStatus);

  std::future<NetworkStatus> nextStatus;
};
