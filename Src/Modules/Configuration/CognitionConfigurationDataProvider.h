/**
* @file CognitionConfigurationDataProvider.h
* This file declares a module that provides data loaded from configuration files.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/USBStatus.h"
#include "Representations/Infrastructure/USBSettings.h"
#include "Tools/ProcessFramework/CycleLocal.h"
#include <memory>

MODULE(CognitionConfigurationDataProvider,
  USES(OwnTeamInfo),
  REQUIRES(USBStatus),
  PROVIDES_WITHOUT_MODIFY(FieldDimensions),
  PROVIDES(RobotDimensions),
  PROVIDES(HeadLimits),
  PROVIDES_WITHOUT_MODIFY(OdometryCorrectionTables),
  PROVIDES(USBSettings)
);

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase
{
private:
  static CycleLocal<CognitionConfigurationDataProvider*> theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  std::unique_ptr<FieldDimensions> theFieldDimensions = nullptr;
  std::unique_ptr<CameraCalibration> theCameraCalibration = nullptr;
  std::unique_ptr<RobotDimensions> theRobotDimensions = nullptr;
  std::unique_ptr<HeadLimits> theHeadLimits = nullptr;
  std::unique_ptr<OdometryCorrectionTables> theOdometryCorrectionTables = nullptr;

  unsigned lastMountTimestamp = 0;

  void update(USBSettings& usbSettings);
  void update(FieldDimensions& fieldDimensions);
  void update(CameraCalibration& cameraCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(HeadLimits& headLimits);
  void update(OdometryCorrectionTables& odometryCorrectionTables);

  void readFieldDimensions();
  void readCameraCalibration();
  void readRobotDimensions();
  void readHeadLimits();
  void readOdometryCorrectionTables();

public:
  /**
  * Default constructor.
  */
  CognitionConfigurationDataProvider();

  /**
  * Destructor.
  */
  ~CognitionConfigurationDataProvider();

  /**
   * Called from a MessageQueue to distribute messages
   * @param message The message that can be read
   * @return true if the message was handled
   */
  static bool handleMessage(InMessage& message);
};
