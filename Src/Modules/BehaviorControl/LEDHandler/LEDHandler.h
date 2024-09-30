/**
* @file LEDHandler.h
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/ColorModelConversions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Infrastructure/CMCorrectorStatus.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/MotionControl/StandEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(LEDHandler,
  REQUIRES(BallSymbols),
  REQUIRES(BehaviorData),
  REQUIRES(CameraCalibration),
  REQUIRES(RoleSymbols),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SpeedInfo),
  REQUIRES(SystemSensorData),
  REQUIRES(TeammateData),
  REQUIRES(WhistleDortmund),
  REQUIRES(StandEngineOutput),
  REQUIRES(CMCorrectorStatus),
  REQUIRES(FallDownState),
  REQUIRES(MotionInfo),
  REQUIRES(WalkCalibration),
  PROVIDES(LEDRequest)
);

class LEDHandler : public LEDHandlerBase
{
private:
  void update(LEDRequest& ledRequest);

  void setCalibrationLEDs(LEDRequest& ledRequest) const;
  void setTestLEDs(LEDRequest& ledRequest) const;
  void setWhistleDetectionLEDs(LEDRequest& ledRequest) const;
  void setGameLEDs(LEDRequest& ledRequest);

  void setStaticFireEyes(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED col3);
  void setRandomizedFireEyes(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED col3);
  void setFloatingFireEyes(LEDRequest& ledRequest, short hue1, short hue2, short hue3);
  void setStaticRainbowEyes(LEDRequest& ledRequest);
  void setFloatingRainbowEyes(LEDRequest& ledRequest);

  // duration means number of update calls
  void setRotatingEyesTwoColors(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, int duration, bool mirrored);
  void setRotatingEyesThreeColors(LEDRequest& ledRequest, LEDRequest::RGBLED col1, LEDRequest::RGBLED col2, LEDRequest::RGBLED transition, int duration, bool mirrored);

  void setBatteryInfo(LEDRequest& ledRequest) const; /**< Battery info to right ear. */
  void setChargingStatus(LEDRequest& ledRequest); /**< Set charging status to eyes. */
  void setCheeringAnimation(LEDRequest& ledRequest); /**< Set cheering animation to eyes. */
  void setConnectionInfo(LEDRequest& ledRequest) const; /**< Connection info to left ear. */
  void setDetectionInfo(LEDRequest& ledRequest) const; /**< Set perception info to left eye. */
  void setRoleInfo(LEDRequest& ledRequest) const; /**< Set role info to right eye. */
  void setFlyingStateInfo(LEDRequest& ledRequest); /**< Set feedback for flying state to eyes. */
  void setDamagedJoints(LEDRequest& ledRequest); /**< Set feedback for broken Joints to eyes. */
  void setGameStateInfo(LEDRequest& ledRequest); /**< Set game state info to chest button. */
  void setLocaInfo(LEDRequest& ledRequest) const; /**< Set loca info on head. */
  void setMotionInfo(LEDRequest& ledRequest) const; /**< Set motion info to feet. */
  void setMicStatusLEDs(LEDRequest& ledRequest); /**<show mic status during initial. */

  float blinking = 0.f;
  float fastBlinking = 0.f;

  int rotationState = 0;
  int rotationStateFire = 0;
  int lastUpdate = 0;
  int offset = 0;

  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col1LEDsOld = {0, 0, 0, 1, 1, 1, 0, 0};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col2LEDsOld = {0, 1, 1, 0, 0, 0, 1, 1};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col3LEDsOld = {1, 1, 1, 1, 1, 1, 1, 1};

  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col1LEDs = {0, 0, 0, 1, 1, 1, 0, 0};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col2LEDs = {0, 1, 1, 0, 0, 0, 1, 1};
  std::array<bool, LEDRequest::EyeLED::numOfEyeLEDs> col3LEDs = {1, 1, 1, 1, 1, 1, 1, 1};
};
