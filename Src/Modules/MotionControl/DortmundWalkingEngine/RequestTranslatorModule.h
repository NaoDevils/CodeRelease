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
#include "Modules/MotionControl/DortmundWalkingEngine/FLIPMObserver.h"
#include "Representations/BehaviorControl/GoalSymbols.h"

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
  REQUIRES(GoalSymbols),
  PROVIDES(PatternGenRequest),
  PROVIDES(WalkingEngineParams),
  PROVIDES(FLIPMObserverParams),
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
  void update(FLIPMObserverParams& flipmObserverParams);
private:
  void save(WalkingEngineParams &params, std::string path);
  void save(FLIPMObserverParams &flipmObserverParams, std::string path);
  WalkingInfo theWalkingInfoDummy;
};

