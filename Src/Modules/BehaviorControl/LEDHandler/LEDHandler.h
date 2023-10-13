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
#include "Representations/MotionControl/MotionState.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(LEDHandler,
  REQUIRES(BallModel),
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorData),
  REQUIRES(MotionState),
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
  REQUIRES(FallDownState),
  PROVIDES(LEDRequest)
);

class LEDHandler : public LEDHandlerBase
{
private:
  void update(LEDRequest& ledRequest);

  void setEyeColor(LEDRequest& ledRequest, bool left, BehaviorLEDRequest::EyeColor col, LEDRequest::LEDState s, bool onlyLeft = false, bool onlyRight = false);

  void setFrameworkInactiveLEDs(LEDRequest& ledRequest);
  void setCalibrationLEDs(LEDRequest& ledRequest);
  void setTestLEDs(LEDRequest& ledRequest);
  void setWhistleDetectionLEDs(LEDRequest& ledRequest);
  void setGameLEDs(LEDRequest& ledRequest);
  void setStaticFireEyes(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor col3);
  void setRandomizedFireEyes(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor col3);
  void setDynamicRainbowEyes(LEDRequest& ledRequest);
  void setStaticRainbowEyes(LEDRequest& ledRequest);
  // duration means number of update calls
  void setRotatingEyesTwoColors(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, int duration, bool mirrored);
  void setRotatingEyesThreeColors(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col1, BehaviorLEDRequest::EyeColor col2, BehaviorLEDRequest::EyeColor transition, int duration, bool mirrored);
  // led must be between 0 and 7: 0=N, 1=NW, 2=W, 3=SW, 4=S, 5=SE, 6=E, 7=NE
  void setLEDInBothEyesToColor(LEDRequest& ledRequest, BehaviorLEDRequest::EyeColor col, int led, bool mirrored);
  void setBatteryInfo(LEDRequest& ledRequest); /**< Battery info to right ear. */
  void setConnectionInfo(LEDRequest& ledRequest); /**< Connection info to left ear. */
  void setDetectionInfo(LEDRequest& ledRequest); /**< Set perception info to left eye. */
  void setRoleInfo(LEDRequest& ledRequest); /**< Set role info to right eye. */
  void setFlyingStateInfo(LEDRequest& ledRequest); /**< Set feedback for flying state to eyes. */
  void setDamagedJoints(LEDRequest& ledRequest); /**< Set feedback for broken Joints to eyes. */
  void setGameStateInfo(LEDRequest& ledRequest); /**< Set game state info to chest button. */
  void setLocaInfo(LEDRequest& ledRequest); /**< Set loca info on head. */
  void setMotionInfo(LEDRequest& ledRequest); /**< Set motion info to feet. */
  void setObstacleInfo(LEDRequest& ledRequest); /**< Set obstacle info to feet. */

  int rotationState = 0;
  int lastUpdate = 0;
  std::vector<bool> col1LEDs = {0, 0, 0, 0, 1, 1, 1, 0};
  std::vector<bool> col2LEDs = {0, 1, 1, 1, 0, 0, 0, 1};
  std::vector<bool> col3LEDs = {1, 1, 1, 1, 1, 1, 1, 1};
};
