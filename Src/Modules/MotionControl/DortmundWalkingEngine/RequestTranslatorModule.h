#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Tools/Module/Module.h"
#include "RequestTranslator.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/ArmMovement.h"

MODULE(RequestTranslatorModule,
{ ,
  USES(WalkingInfo),
  REQUIRES(MotionSelection),
  REQUIRES(FallDownState),
  REQUIRES(SpeedRequest),
  REQUIRES(InertialSensorData),
  REQUIRES(BallModelAfterPreview),
  USES(ArmMovement),
  USES(SpeedInfo),
  PROVIDES(PatternGenRequest),
  PROVIDES(WalkingEngineParams),
  PROVIDES(FreeLegPhaseParams),
});

class RequestTranslatorModule : public RequestTranslatorModuleBase
{
public:
  RequestTranslatorModule();

protected:
  MotionSelection filteredMotionSelection;
	RequestTranslator translator;

	void update(PatternGenRequest& patternGenRequest);
	void update(WalkingEngineParams& walkingEngineParams);
	void update(ControllerParams& controllerParams);

	void update(FreeLegPhaseParams& freeLegPhaseParams);
private:
  void save(WalkingEngineParams &params, std::string path);
  WalkingInfo theWalkingInfoDummy;
};

