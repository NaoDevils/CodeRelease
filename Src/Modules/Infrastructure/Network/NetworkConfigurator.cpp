#include "NetworkConfigurator.h"
#include <nlohmann/json.hpp>
#include <iostream>

void NetworkConfigurator::update(NetworkStatus& networkStatus)
{
  if (nextStatus.valid())
  {
    if (nextStatus.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      networkStatus = nextStatus.get();

      if (networkStatus.status == NetworkStatus::Status::configured)
        OUTPUT_TEXT("NetworkConfigurator: Configured!");
      else
        OUTPUT_WARNING("NetworkConfigurator: Failed!");
    }
  }

  if (!nextStatus.valid() && networkStatus.configureTimestamp != theUSBSettings.updateTimestamp)
  {
    if (theUSBSettings.ip.empty() && theUSBSettings.wifiSSID.empty())
      return;

    networkStatus.status = NetworkStatus::Status::configuring;

    using namespace nlohmann;

    json j = {{"dhcp4", false}, {"dhcp6", false}, {"optional", true}};

    if (!theUSBSettings.ip.empty())
      j["addresses"] = {theUSBSettings.ip};

    if (!theUSBSettings.wifiSSID.empty())
    {
      json ap = json::object();
      if (!theUSBSettings.wifiPassword.empty())
        ap["password"] = theUSBSettings.wifiPassword;

      j["access-points"] = {{theUSBSettings.wifiSSID, ap}};
    }

    // replace ' with '"'"'
    std::string output = j.dump();
    const std::string search = "'";
    const std::string replace = "'\"'\"'";
    for (std::size_t pos = output.find(search); pos != std::string::npos; pos = output.find(search, pos))
    {
      output = output.replace(pos, search.length(), replace);
      pos += replace.length();
    }

    const std::string command = "sudo /usr/sbin/configure-network '" + output + "'";

    nextStatus = std::async(std::launch::async,
        [this, command]()
        {
          NetworkStatus status;
          status.configureTimestamp = theUSBSettings.updateTimestamp;
          status.status = system(command.c_str()) == 0 ? NetworkStatus::Status::configured : NetworkStatus::Status::failed;
          return status;
        });
  }
}

MAKE_MODULE(NetworkConfigurator, cognitionInfrastructure)
