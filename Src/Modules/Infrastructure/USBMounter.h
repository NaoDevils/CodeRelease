#pragma once

#include <string>
#include <future>
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/USBStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(USBMounter,
  USES(RobotInfo),
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  REQUIRES(USBStatus),
  PROVIDES(USBStatus),
  LOADS_PARAMETERS(,
    (std::string)("") mountPath,
    (std::string)("logs") logDirectory,
    (unsigned)(5) unmountRetries
  )
);

class USBMounter : public USBMounterBase
{
public:
  USBMounter();
  //~USBMounter();

private:
  void update(USBStatus& usbStatus);

  void formatBehavior();
  USBStatus::MountStatus mount() const;
  USBStatus::MountStatus unmount() const;
  USBStatus::MountStatus checkStatus() const;

  std::future<USBStatus::MountStatus> nextMountStatus;
  USBStatus::MountStatus initialStatus = USBStatus::MountStatus::inactive;
  bool active = false;

  unsigned keyPressedTime = 0;
  unsigned keyReleasedTime = 0;
  bool saidFormatUsb = false;
};
