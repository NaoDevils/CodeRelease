/**
* @file HeadControl.h
*
* Declaration of class HeadControl, 
* calculates optimal head joint angle depending on HeadPOIList.
* 
* @author <a href="mailto:mahdokht.mohammadi@tu-dortmund.de">Mahdokht Mohammadi</a>
* @author <a href="mailto:aaron.larisch@tu-dortmund.de">Aaron Larisch</a>
*
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Range.h"

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BallSymbols.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/BehaviorControl/HeadPOIList.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/RemoteBallModel.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/MotionControl/HeadAngleRequest.h"

#include <optional>


MODULE(HeadControl,
  REQUIRES(BallSymbols),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(CameraInfoUpper),
  REQUIRES(HeadPOIList),
  REQUIRES(JointSensorData),
  REQUIRES(RemoteBallModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TorsoMatrix),
  PROVIDES(HeadAngleRequest),
  LOADS_PARAMETERS(
    ,
    (float)(550.f) minDistanceForLowerCamera,
    (float)(650.f) maxDistanceForLowerCamera,
    (unsigned)(200) focusTime,
    (Angle)(2_deg) focusAngleReached,
    (Angle)(10_deg) sweepAngleReached,
    (Angle)(80_deg) defaultSpeed,
    (Vector3f)({50.f, 0.f, 0.f}) minTiltObject,
    (Vector3f)({1000.f, 0.f, 580.f}) maxTiltObject
  )
);

class HeadControl : public HeadControlBase
{
  void update(HeadAngleRequest& headAngleRequest);
  std::vector<Vector2a> getHeadAngles(const HeadPOIList::Target& target) const;

  Vector2a transformHeadPosition(const Vector3f& headpos, bool lowerCamera, Angle imageTilt = 0_deg, Angle imagePan = 0_deg) const;

  std::vector<Vector2a> getNextTarget();

  Rangea tiltLimits{0_deg, 0_deg};

  bool movingLeft = true;
  size_t lastCameraPosition = 0;
  size_t currentTarget = 0;
};
