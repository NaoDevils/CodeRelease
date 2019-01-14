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
#include "Representations/Perception/CLIPGoalPercept.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/BehaviorControl/BehaviorData.h"
#include "Representations/BehaviorControl/RoleSymbols.h"
#include "Representations/BehaviorControl/BehaviorLEDRequest.h"
#include "Representations/Modeling/WhistleDortmund.h"
#include "Representations/BehaviorControl/GameSymbols.h"
#include "Representations/MotionControl/StandEngineOutput.h"

MODULE(LEDHandler,
{ ,
  REQUIRES(BallModel),
  REQUIRES(BehaviorData),
  REQUIRES(BehaviorLEDRequest),
  REQUIRES(RoleSymbols),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(CLIPGoalPercept),
  REQUIRES(GroundContactState),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SystemSensorData),
  REQUIRES(TeammateData),
  REQUIRES(WhistleDortmund),
  REQUIRES(GameSymbols),
  REQUIRES(StandEngineOutput),
  PROVIDES(LEDRequest),
});

class LEDHandler : public LEDHandlerBase
{
private:
  void update(LEDRequest& ledRequest);

  void setEyeColor(LEDRequest& ledRequest,
                   bool left,
                   BehaviorLEDRequest::EyeColor col,
                   LEDRequest::LEDState s);

  void setRightEar(LEDRequest& ledRequest);
  void setLeftEar(LEDRequest& ledRequest);
  void setLeftEye(LEDRequest& ledRequest);
  void setRightEye(LEDRequest& ledRequest);
  void setHead(LEDRequest& ledRequest);
  void setChestButton(LEDRequest& ledRequest);
  void setFeet(LEDRequest& ledRequest);
};
