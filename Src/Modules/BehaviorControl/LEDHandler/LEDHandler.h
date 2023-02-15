/**
* @file LEDHandler.h
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/CMCorrectorStatus.h"
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/MotionControl/StandEngineOutput.h"

MODULE(LEDHandler,
  REQUIRES(BallModel),
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorData),
  REQUIRES(BehaviorLEDRequest),
  REQUIRES(RoleSymbols),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(CLIPGoalPercept),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SpeedInfo),
  REQUIRES(SystemSensorData),
  REQUIRES(TeammateData),
  REQUIRES(WhistleDortmund),
  REQUIRES(GameSymbols),
  REQUIRES(StandEngineOutput),
  REQUIRES(CMCorrectorStatus),
  PROVIDES(LEDRequest)
);

class LEDHandler : public LEDHandlerBase
{
private:
  void update(LEDRequest& ledRequest);

  void setEyeColor(LEDRequest& ledRequest, bool left, BehaviorLEDRequest::EyeColor col, LEDRequest::LEDState s, bool onlyLeft = false, bool onlyRight = false);

  void setFrameworkInactiveLEDs(LEDRequest& ledRequest);
  void setCalibrationLEDs(LEDRequest& ledRequest);
  void setWhistleDetectionLEDs(LEDRequest& ledRequest);
  void setGameLEDs(LEDRequest& ledRequest);
  void setBatteryInfo(LEDRequest& ledRequest); /**< Battery info to right ear. */
  void setConnectionInfo(LEDRequest& ledRequest); /**< Connection info to left ear. */
  void setDetectionInfo(LEDRequest& ledRequest); /**< Set perception info to left eye. */
  void setRoleInfo(LEDRequest& ledRequest); /**< Set role info to right eye. */
  void setGameStateInfo(LEDRequest& ledRequest); /**< Set game state info to chest button. */
  void setLocaInfo(LEDRequest& ledRequest); /**< Set loca info on head. */
  void setMotionInfo(LEDRequest& ledRequest); /**< Set motion info to feet. */
  void setObstacleInfo(LEDRequest& ledRequest); /**< Set obstacle info to feet. */
};
