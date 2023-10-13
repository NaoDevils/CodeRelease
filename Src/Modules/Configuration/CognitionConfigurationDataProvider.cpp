/**
* @file CognitionConfigurationDataProvider.cpp
* This file implements a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include <cstdio>

#ifdef TARGET_ROBOT
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "Platform/Linux/NaoBodyV6.h"
#endif

#include "CognitionConfigurationDataProvider.h"
#include "Platform/File.h"

CycleLocal<CognitionConfigurationDataProvider*> CognitionConfigurationDataProvider::theInstance(nullptr);

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider()
{
  theInstance = this;

  readFieldDimensions();
  readCameraCalibration();
  readRobotDimensions();
  readHeadLimits();
  readOdometryCorrectionTables();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider()
{
  theInstance.reset();
}

void CognitionConfigurationDataProvider::update(USBSettings& usbSettings)
{
#ifdef TARGET_ROBOT
  if ((theUSBStatus.status == USBStatus::MountStatus::readOnly || theUSBStatus.status == USBStatus::MountStatus::readWrite) && theUSBStatus.mountTimestamp != usbSettings.updateTimestamp)
  {
    usbSettings.updateTimestamp = theUSBStatus.mountTimestamp;

    try
    {
      using namespace nlohmann;

      const std::string filename(theUSBStatus.path + "/settings.json");
      std::ifstream ifs(filename);
      if (ifs.fail())
        return;

      const json j = json::parse(ifs);

      if (j.contains("wifi"))
      {
        const json& wifi = j["wifi"];
        if (wifi.contains("ssid") && wifi.contains("password"))
        {
          usbSettings.wifiSSID = wifi["ssid"].get<std::string>();
          usbSettings.wifiPassword = wifi["password"].get<std::string>();
        }
      }

      if (j.contains("team"))
      {
        const json& team = j["team"];
        usbSettings.teamNumber = team.value("number", 0);
        usbSettings.teamPort = team.value("port", 0);
      }

      const std::string headName = Global::getSettings().robotName;
      const std::string headId = NaoBodyV6().getHeadId();
      const std::string bodyId = NaoBodyV6().getBodyId();

      const json& robots = j.at("robots");
      const json& robot = [&]
      {
        if (robots.contains(headName))
          return robots[headName];
        else if (robots.contains(headId))
          return robots[headId];
        else if (robots.contains(bodyId))
          return robots[bodyId];
        else if (robots.contains("Nao"))
          return robots["Nao"];
        else
          throw std::invalid_argument("Robot not found");
      }();

      usbSettings.ip = robot.value("ip", "");
      usbSettings.robotNumber = robot.value("number", 0);
    }
    catch (const std::exception& e)
    {
      OUTPUT_WARNING("CognitionConfigurationDataProvider: Malformed settings.json: " << e.what());
    }
  }
#endif
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
{
  if (theFieldDimensions)
  {
    fieldDimensions = *theFieldDimensions;
    theFieldDimensions.reset();
  }

  if ((theUSBStatus.status == USBStatus::MountStatus::readOnly || theUSBStatus.status == USBStatus::MountStatus::readWrite) && lastMountTimestamp != theUSBStatus.mountTimestamp)
  {
    lastMountTimestamp = theUSBStatus.mountTimestamp;
    fieldDimensions.loadFromJsonFile(theUSBStatus.path + "/field_dimensions.json");
  }

  fieldDimensions.drawPolygons(theOwnTeamInfo.fieldPlayerColour);
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration)
{
  if (theCameraCalibration)
  {
    cameraCalibration = *theCameraCalibration;
    theCameraCalibration.reset();
  }
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if (theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    theRobotDimensions.reset();
  }
}

void CognitionConfigurationDataProvider::update(HeadLimits& headLimits)
{
  if (theHeadLimits)
  {
    headLimits = *theHeadLimits;
    theHeadLimits.reset();
  }
}

void CognitionConfigurationDataProvider::update(OdometryCorrectionTables& odometryCorrectionTables)
{
  if (theOdometryCorrectionTables)
  {
    odometryCorrectionTables = *theOdometryCorrectionTables;
    theOdometryCorrectionTables.reset();
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = std::make_unique<FieldDimensions>();

  // try .json first, fallback to .cfg
  if (!theFieldDimensions->loadFromJsonFile(std::string(File::getBHDir()) + "/Config/field_dimensions.json"))
    theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraCalibration()
{
  ASSERT(!theCameraCalibration);

  InMapFile stream("cameraCalibration.cfg");
  if (stream.exists())
  {
    theCameraCalibration = std::make_unique<CameraCalibration>();
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if (stream.exists())
  {
    theRobotDimensions = std::make_unique<RobotDimensions>();
    stream >> *theRobotDimensions;
  }
}

void CognitionConfigurationDataProvider::readHeadLimits()
{
  ASSERT(!theHeadLimits);

  InMapFile stream("headLimits.cfg");
  if (stream.exists())
  {
    theHeadLimits = std::make_unique<HeadLimits>();
    stream >> *theHeadLimits;
  }
}

void CognitionConfigurationDataProvider::readOdometryCorrectionTables()
{
  ASSERT(!theOdometryCorrectionTables);

  InMapFile stream("odometryCorrectionTables.cfg");
  if (stream.exists())
  {
    theOdometryCorrectionTables = std::make_unique<OdometryCorrectionTables>();
    stream >> *theOdometryCorrectionTables;
  }
}

bool CognitionConfigurationDataProvider::handleMessage(InMessage& message)
{
  return false;
}

MAKE_MODULE(CognitionConfigurationDataProvider, cognitionInfrastructure)
