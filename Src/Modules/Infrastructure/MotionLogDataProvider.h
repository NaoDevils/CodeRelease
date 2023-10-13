/**
 * @file MotionLogDataProvider.h
 * This file declares a module that provides data replayed from a log file.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/SensorData/SonarSensorData.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/CustomStepSelection.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionState.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Module/Module.h"
#include "LogDataProvider.h"
#include "Tools/ProcessFramework/CycleLocal.h"

MODULE(MotionLogDataProvider,
  PROVIDES(CustomStepSelection),
  PROVIDES(FallDownState),
  PROVIDES(Footpositions),
  PROVIDES(FrameInfo),
  PROVIDES(FsrSensorData),
  PROVIDES(GroundTruthOdometryData),
  PROVIDES(InertialData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointAngles),
  PROVIDES(JointRequest),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(KinematicRequest),
  PROVIDES(MotionInfo),
  PROVIDES(MotionState),
  PROVIDES(MotionRequest),
  PROVIDES(MotionSelection),
  PROVIDES(OdometryData),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RobotInfo),
  PROVIDES(SpeedRequest),
  PROVIDES(SystemSensorData),
  PROVIDES(SonarSensorData),
  PROVIDES(WalkCalibration)
);

class MotionLogDataProvider : public MotionLogDataProviderBase, public LogDataProvider
{
private:
  static CycleLocal<MotionLogDataProvider*> theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
  bool frameDataComplete; /**< Were all messages of the current frame received? */
  OdometryData lastOdometryData;

  /**
   * The method is called for every incoming debug message by handleMessage.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  bool handleMessage2(InMessage& message);

public:
  MotionLogDataProvider();
  ~MotionLogDataProvider();

  void update(CustomStepSelection&) {}
  void update(FallDownState&) {}
  void update(Footpositions&) {}
  void update(FrameInfo&) {}
  void update(FsrSensorData&) {}
  void update(GroundTruthOdometryData&);
  void update(InertialData&) {}
  void update(InertialSensorData&) {}
  void update(JointAngles&) {}
  void update(JointRequest&) {}
  void update(JointSensorData&) {}
  void update(KinematicRequest&) {}
  void update(KeyStates&) {}
  void update(MotionInfo&) {}
  void update(MotionRequest&) {}
  void update(MotionSelection&) {}
  void update(MotionState&) {}
  void update(OdometryData&) {}
  void update(OpponentTeamInfo&) {}
  void update(OwnTeamInfo&) {}
  void update(RobotInfo&) {}
  void update(SpeedRequest&) {}
  void update(SystemSensorData&) {}
  void update(SonarSensorData&) {}
  void update(WalkCalibration&) {}

  /**
   * The method is called for every incoming debug message.
   * @param message An interface to read the message from the queue.
   * @return Was the message handled?
   */
  static bool handleMessage(InMessage& message);

  /**
   * The method returns whether idProcessFinished was received.
   * @return Were all messages of the current frame received?
   */
  static bool isFrameDataComplete();
};
