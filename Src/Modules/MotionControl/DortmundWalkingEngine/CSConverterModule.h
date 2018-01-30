/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CSConverterModule.h
* Module wrapper for the CSConverter
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "CSConverter.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/ArmContact.h"
#include "Representations/Configuration/RobotDimensions.h"

MODULE(CSConverterModule,
{ ,
  REQUIRES(TargetCoM),
  REQUIRES(WalkingEngineParams),
  REQUIRES(ControllerParams),
  REQUIRES(ActualCoMRCS),
  REQUIRES(BodyTilt),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotModel),
  REQUIRES(RobotInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(InertialSensorData),
  REQUIRES(FallDownState),
  REQUIRES(Footpositions),
  REQUIRES(FootSteps),
  REQUIRES(ArmContact),
  REQUIRES(RobotDimensions),
  REQUIRES(WalkCalibration),
  PROVIDES(KinematicRequest),
  PROVIDES(WalkingInfo),
  PROVIDES(FixedOdometryRobotPose),
});

class CSConverterModule : public CSConverterModuleBase
{
public:
	CSConverterModule();

	CSConverter converter;

	void update(KinematicRequest &kinematicRequest);
	void update(WalkingInfo &walkingInfo);
	void update(FixedOdometryRobotPose &fixedOdometryRobotPose);
};



