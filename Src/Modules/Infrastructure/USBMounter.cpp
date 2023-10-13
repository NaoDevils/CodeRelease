#include "USBMounter.h"
#include "Tools/Build.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <filesystem>

USBMounter::USBMounter()
{
  if constexpr (Build::targetRobot())
  {
    initialStatus = checkStatus();
  }
}

// Sadly, we cannot unmount here because logger does not close its file at this point.
//USBMounter::~USBMounter()
//{
//  USBStatus::MountStatus status = theUSBStatus.status;
//
//  const bool mountOperation = status == USBStatus::MountStatus::mounting || status == USBStatus::MountStatus::unmounting;
//  if (mountOperation && nextMountStatus.valid()
//      && nextMountStatus.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
//    status = nextMountStatus.get();
//
//  if (status == USBStatus::MountStatus::readOnly || status == USBStatus::MountStatus::readWrite)
//    unmount();
//}

void USBMounter::update(USBStatus& usbStatus)
{
  if constexpr (!Build::targetRobot())
  {
    usbStatus.status = USBStatus::MountStatus::notMounted;
  }
  else
  {

    usbStatus.path = mountPath;
    usbStatus.logPath = mountPath + "/" + logDirectory;

    // apply initial status from constructor
    if (initialStatus != USBStatus::MountStatus::inactive)
    {
      usbStatus.status = initialStatus;
      initialStatus = USBStatus::MountStatus::inactive;
    }

    // ignores transitionToFramework = 1 during initialization
    if (theRobotInfo.transitionToFramework == 0.f)
      active = true;
    if (!active)
      return;

    formatBehavior();

    switch (usbStatus.status)
    {
    case USBStatus::MountStatus::inactive:
      if (theRobotInfo.transitionToFramework > 0.f)
      {
        usbStatus.status = USBStatus::MountStatus::mounting;
        nextMountStatus = std::async(std::launch::async, &USBMounter::mount, this);
      }
      break;
    case USBStatus::MountStatus::mounting:
    case USBStatus::MountStatus::unmounting:
      if (nextMountStatus.valid() && nextMountStatus.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        usbStatus.status = nextMountStatus.get();

      if (usbStatus.status == USBStatus::MountStatus::readOnly || usbStatus.status == USBStatus::MountStatus::readWrite)
        usbStatus.mountTimestamp = theFrameInfo.time;
      break;
    case USBStatus::MountStatus::readOnly:
    case USBStatus::MountStatus::readWrite:
      if (theRobotInfo.transitionToFramework == 0.f)
      {
        usbStatus.status = USBStatus::MountStatus::unmounting;
        nextMountStatus = std::async(std::launch::async, &USBMounter::unmount, this);
      }
      break;
    case USBStatus::MountStatus::notMounted:
      if (theRobotInfo.transitionToFramework == 0.f)
      {
        usbStatus.status = USBStatus::MountStatus::inactive;
      }
      break;
    case USBStatus::MountStatus::unknown:
      // TODO: handling
      break;
    }
  }
}

void USBMounter::formatBehavior()
{
  const bool keyCombinationPressed = theKeyStates.pressed[KeyStates::Key::headFront] && !theKeyStates.pressed[KeyStates::Key::headMiddle] && theKeyStates.pressed[KeyStates::Key::headRear];

  (keyCombinationPressed ? keyReleasedTime : keyPressedTime) = theFrameInfo.time;

  if (theUSBStatus.status != USBStatus::MountStatus::inactive || theRobotInfo.transitionToFramework > 0.f)
  {
    saidFormatUsb = false;
    return;
  }

  if (theFrameInfo.getTimeSince(keyPressedTime) > 5000)
  {
    if (!saidFormatUsb)
      SystemCall::text2Speech("Format U S B?");
    saidFormatUsb = true;
  }
  if (theFrameInfo.getTimeSince(keyReleasedTime) > 3000)
  {
    if (saidFormatUsb)
      SystemCall::text2Speech("Aborted");
    saidFormatUsb = false;
  }

  if (saidFormatUsb && theKeyStates.pressed[KeyStates::Key::headMiddle])
  {
    SystemCall::text2Speech("Confirmed");
    saidFormatUsb = false;
    if (system("/usr/bin/format_usb") == 0)
      SystemCall::text2Speech("Finished");
  }
}

USBStatus::MountStatus USBMounter::checkStatus() const
{
  int ret = system(("mountpoint -q \"" + mountPath + "\"").c_str());

  if (ret != 0)
    return USBStatus::MountStatus::notMounted;

  std::cout << "USBMounter: Status: USB stick is mounted!" << std::endl;

  const std::string logPath = mountPath + "/" + logDirectory;

  try
  {
    std::filesystem::create_directory(logPath);
  }
  catch (...)
  {
    std::cerr << "USBMounter: Status: Cannot create log directory!" << std::endl;
    return USBStatus::MountStatus::readOnly;
  }

  USBStatus::MountStatus status;

  const std::string testfile = logPath + "/write.test";
  OutBinaryFile file = OutBinaryFile(testfile);
  if (file.exists())
  {
    std::cout << "USBMounter: Status: Write test successful!" << std::endl;
    status = USBStatus::MountStatus::readWrite;
  }
  else
  {
    std::cerr << "USBMounter: Status: Write test failed!" << std::endl;
    status = USBStatus::MountStatus::readOnly;
  }

  try
  {
    std::filesystem::remove(testfile);
  }
  catch (...)
  {
  }

  return status;
}

USBStatus::MountStatus USBMounter::mount() const
{
  if (const USBStatus::MountStatus status = checkStatus(); status == USBStatus::MountStatus::readOnly || status == USBStatus::MountStatus::readWrite)
    return status;

  std::cout << "USBMounter: Trying to mount USB stick..." << std::endl;

  try
  {
    std::filesystem::create_directory(mountPath);
  }
  catch (...)
  {
    std::cerr << "USBMounter: Cannot create mount directory!" << std::endl;
    return USBStatus::MountStatus::notMounted;
  }

  const int ret = system(("mount \"" + mountPath + "\"").c_str());

  if (ret == 0)
  {
    std::cout << "USBMounter: Mounting successful!" << std::endl;
    return checkStatus();
  }
  else
  {
    std::cerr << "USBMounter: Mounting failed!" << std::endl;
    return USBStatus::MountStatus::notMounted;
  }
}

USBStatus::MountStatus USBMounter::unmount() const
{
  USBStatus::MountStatus status = checkStatus();
  for (unsigned i = 0; i < unmountRetries; ++i)
  {
    if (status == USBStatus::MountStatus::notMounted)
      break;

    std::cout << "USBMounter: Unmounting USB stick..." << std::endl;

    int ret = system(("umount \"" + mountPath + "\"").c_str());

    if (ret == 0)
    {
      std::cout << "USBMounter: USB stick unmounted." << std::endl;
      status = checkStatus();
    }
    else
    {
      std::cout << "USBMounter: USB stick umounting failed!" << std::endl;
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(3000ms);
      status = USBStatus::MountStatus::unknown;
    }
  }

  if (status == USBStatus::MountStatus::notMounted)
  {
    SystemCall::text2Speech("U S B unmounted");
    return USBStatus::MountStatus::inactive;
  }
  else
  {
    return status;
  }
}


MAKE_MODULE(USBMounter, cognitionInfrastructure)
