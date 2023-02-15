/**
 * @file MotionConfigurationDataProvider.cpp
 * This file implements a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionConfigurationDataProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Platform/File.h"

MAKE_MODULE(MotionConfigurationDataProvider, motionInfrastructure)

MotionConfigurationDataProvider::MotionConfigurationDataProvider()
{
  readFieldDimensions();
  readJointCalibration();
  readJointDeCalibration();
  readMassCalibration();
  readMotionSettings();
  readOdometryCorrectionTables();
  readRobotDimensions();
  readStiffnessSettings();
  readUsConfiguration();
}

void MotionConfigurationDataProvider::update(FieldDimensions& fieldDimensions)
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
}

void MotionConfigurationDataProvider::update(JointCalibration& jointCalibration)
{
  if (theJointCalibration)
  {
    jointCalibration = *theJointCalibration;
    theJointCalibration.reset();
  }
  DEBUG_RESPONSE_ONCE("representation:JointCalibration:once") OUTPUT(idJointCalibration, bin, jointCalibration);
}

void MotionConfigurationDataProvider::update(JointDeCalibration& jointDeCalibration)
{
  if (theJointDeCalibration)
  {
    jointDeCalibration = *theJointDeCalibration;
    theJointDeCalibration.reset();
  }
}

void MotionConfigurationDataProvider::update(MassCalibration& massCalibration)
{
  if (theMassCalibration)
  {
    massCalibration = *theMassCalibration;
    theMassCalibration.reset();
  }
}

void MotionConfigurationDataProvider::update(MotionSettings& motionSettings)
{
  if (theMotionSettings)
  {
    motionSettings = *theMotionSettings;
    theMotionSettings.reset();
  }
}

void MotionConfigurationDataProvider::update(OdometryCorrectionTables& odometryCorrectionTables)
{
  if (theOdometryCorrectionTables)
  {
    odometryCorrectionTables = *theOdometryCorrectionTables;
    theOdometryCorrectionTables.reset();
  }
}

void MotionConfigurationDataProvider::update(RobotDimensions& robotDimensions)
{
  if (theRobotDimensions)
  {
    robotDimensions = *theRobotDimensions;
    theRobotDimensions.reset();
  }
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions:once") OUTPUT(idRobotDimensions, bin, robotDimensions);
}

void MotionConfigurationDataProvider::update(StiffnessSettings& stiffnessSettings)
{
  if (theStiffnessSettings)
  {
    stiffnessSettings = *theStiffnessSettings;
    theStiffnessSettings.reset();
  }
}

void MotionConfigurationDataProvider::update(UsConfiguration& usConfiguration)
{
  if (theUsConfiguration)
  {
    usConfiguration = *theUsConfiguration;
    theUsConfiguration.reset();
  }
}

void MotionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = std::make_unique<FieldDimensions>();

  // try .json first, fallback to .cfg
  if (!theFieldDimensions->loadFromJsonFile(std::string(File::getBHDir()) + "/Config/field_dimensions.json"))
    theFieldDimensions->load();
}

void MotionConfigurationDataProvider::readJointCalibration()
{
  ASSERT(!theJointCalibration);

  InMapFile stream("jointCalibration.cfg");
  if (stream.exists())
  {
    theJointCalibration = std::make_unique<JointCalibration>();
    stream >> *theJointCalibration;
  }
}

void MotionConfigurationDataProvider::readJointDeCalibration()
{
  ASSERT(!theJointDeCalibration);

  InMapFile stream("jointDeCalibration.cfg");
  if (stream.exists())
  {
    theJointDeCalibration = std::make_unique<JointDeCalibration>();
    stream >> *theJointDeCalibration;
  }
}

void MotionConfigurationDataProvider::readMassCalibration()
{
  ASSERT(!theMassCalibration);

  InMapFile stream("massCalibration.cfg");
  if (stream.exists())
  {
    theMassCalibration = std::make_unique<MassCalibration>();
    stream >> *theMassCalibration;
  }
}

void MotionConfigurationDataProvider::readMotionSettings()
{
  ASSERT(!theMotionSettings);

  InMapFile stream("motionSettings.cfg");
  if (stream.exists())
  {
    theMotionSettings = std::make_unique<MotionSettings>();
    stream >> *theMotionSettings;
  }
}

void MotionConfigurationDataProvider::readOdometryCorrectionTables()
{
  ASSERT(!theOdometryCorrectionTables);

  InMapFile stream("odometryCorrectionTables.cfg");
  if (stream.exists())
  {
    theOdometryCorrectionTables = std::make_unique<OdometryCorrectionTables>();
    stream >> *theOdometryCorrectionTables;
  }
}

void MotionConfigurationDataProvider::readRobotDimensions()
{
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if (stream.exists())
  {
    theRobotDimensions = std::make_unique<RobotDimensions>();
    stream >> *theRobotDimensions;
  }
}

void MotionConfigurationDataProvider::readStiffnessSettings()
{
  ASSERT(!theStiffnessSettings);

  InMapFile stream("stiffnessSettings.cfg");
  if (stream.exists())
  {
    theStiffnessSettings = std::make_unique<StiffnessSettings>();
    stream >> *theStiffnessSettings;
  }
}

void MotionConfigurationDataProvider::readUsConfiguration()
{
  ASSERT(!theUsConfiguration);

  InMapFile stream("usConfiguration.cfg");
  if (stream.exists())
  {
    theUsConfiguration = std::make_unique<UsConfiguration>();
    stream >> *theUsConfiguration;
  }
}
