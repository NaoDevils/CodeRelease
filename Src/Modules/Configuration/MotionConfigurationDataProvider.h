/**
 * @file MotionConfigurationDataProvider.h
 * This file declares a module that provides data loaded from configuration files.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointDeCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/MotionSettings.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/UsConfiguration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/Infrastructure/USBStatus.h"
#include <memory>

MODULE(MotionConfigurationDataProvider,
  REQUIRES(USBStatus),
  PROVIDES(StiffnessSettings),
  PROVIDES(JointCalibration),
  PROVIDES(JointDeCalibration),
  PROVIDES(MassCalibration),
  PROVIDES(MotionSettings),
  PROVIDES(OdometryCorrectionTables),
  PROVIDES(RobotDimensions),
  PROVIDES(UsConfiguration),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions)
);

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase
{
private:
  std::unique_ptr<StiffnessSettings> theStiffnessSettings = nullptr;
  std::unique_ptr<JointCalibration> theJointCalibration = nullptr;
  std::unique_ptr<JointDeCalibration> theJointDeCalibration = nullptr;
  std::unique_ptr<MassCalibration> theMassCalibration = nullptr;
  std::unique_ptr<MotionSettings> theMotionSettings = nullptr;
  std::unique_ptr<RobotDimensions> theRobotDimensions = nullptr;
  std::unique_ptr<UsConfiguration> theUsConfiguration = nullptr;
  std::unique_ptr<FieldDimensions> theFieldDimensions = nullptr;
  std::unique_ptr<OdometryCorrectionTables> theOdometryCorrectionTables = nullptr;

  unsigned lastMountTimestamp = 0;

  void update(FieldDimensions& fieldDimensions);
  void update(JointCalibration& jointCalibration);
  void update(JointDeCalibration& jointDeCalibration);
  void update(MassCalibration& massCalibration);
  void update(MotionSettings& motionSettings);
  void update(OdometryCorrectionTables& odometryCorrectionTables);
  void update(RobotDimensions& robotDimensions);
  void update(StiffnessSettings& stiffnessSettings);
  void update(UsConfiguration& usConfiguration);

  void readFieldDimensions();
  void readJointCalibration();
  void readJointDeCalibration();
  void readMassCalibration();
  void readMotionSettings();
  void readOdometryCorrectionTables();
  void readRobotDimensions();
  void readStiffnessSettings();
  void readUsConfiguration();

public:
  MotionConfigurationDataProvider();
};
