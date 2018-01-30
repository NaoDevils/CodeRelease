/**
 * @file Modules/MotionControl/DortmundWalkingEngine/RequestTranslator.h
 * Translates the MotionRequest for the WalkingEngine (some clipping and path-calculations)
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 * @author <a href="mailto:stefan.czarnetzki@uni-dortmund.de">Stefan Czarnetzki</a>
 */

#pragma once

#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Tools/Math/Eigen.h"
#include "Point.h"
#include <list>
#include <algorithm>
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Streams/RobotParameters.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/BehaviorControl/GoalSymbols.h"
#include "Tools/Module/ModuleManager.h"
#include "Modules/MotionControl/DortmundWalkingEngine/FLIPMObserver.h"

#define MAX_ACC_WINDOW 100

class RequestTranslator
{
  ROBOT_PARAMETER_CLASS(walkingDecelerate, RequestTranslator)
    PARAM(float, angle_border)  
    PARAM(float, angleSpeedchange)
	  PARAM(bool, disableDynamicSpeed)
    PARAM(float, fPos)
    PARAM(float, fNeg)
	  PARAM(float, var_border)
	  PARAM(unsigned int, standTime)
	  PARAM(int, totalInitTime)
  END_ROBOT_PARAMETER_CLASS(walkingDecelerate)
  
  ROBOT_PARAMETER_CLASS(GoToRequest, RequestTranslator)
    PARAM(float, kpx)
    PARAM(float, kpy)
    PARAM(float, kpr)
    PARAM(float, minSpeedForUsingSpeedSum)
    PARAM(float, maxSpeedSum)
  END_ROBOT_PARAMETER_CLASS(GoToRequest)
  
  friend class RequestTranslatorModule;
public:
	RequestTranslator(const MotionSelection &theMotionSelection,
                      const WalkingInfo &theWalkingInfo,
                      const FallDownState &theFallDownState,
                      const InertialSensorData &theInertialSensorData,
                      const BallModelAfterPreview &theBallModelAfterPreview,
					  const SpeedInfo &theSpeedInfo,
            const SpeedRequest &theSpeedRequest,
                    const ArmMovement &ArmMovement,
    const GoalSymbols &theGoalSymbols
                    );
	~RequestTranslator(void);
	
	void updatePatternGenRequest(PatternGenRequest & patternGenRequest);
	void updateWalkingEngineParams(WalkingEngineParams & walkingEngineParams);
  void updateFLIPMObserverParams(FLIPMObserverParams & flipmObserverParams);
private:
	const MotionSelection & theMotionSelection;
    const WalkingInfo &theWalkingInfo;
	const FallDownState &theFallDownState;
    const InertialSensorData &theInertialSensorData;
    const BallModelAfterPreview &theBallModelAfterPreview;
	const SpeedInfo &theSpeedInfo;
  const SpeedRequest &theSpeedRequest;
  const ArmMovement &theArmMovement;
  const GoalSymbols &theGoalSymbols;
	
  ModuleManager::Configuration config;
  FLIPMObserverParams observerParams;
	WalkingEngineParams walkParams;
	ControllerParams contParams;
	PatternGenRequest::State getNewState(float x, float y, float r);
	FastFilter<float> filter[3];
	PatternGenRequest::State currentState;
	int accCounter[3], lastN;
	bool walkStarted, firstStep;
	float localSensorScale;

  unsigned int timer;
  bool deceleratedByMax;
  bool deceleratedByInstability;
  
  // general goto stuff
  Pose2f gotoPointController(const Pose2f &target, const Pose2f &max);
  Pose2f gotoPointController(const Vector2f &target, const Pose2f &max);

  void clipping(Pose2f &speed);
  Pose2f gotoBallAndStand();
  void reset();
  Pose2f gotoBallAndKick();
 

  float tempMaxSpeed; // dynamicMaximumSpeed
  float theFactor; // factor by which the tempMaxSpeed ist modified
  
  /* 'timer' */
  int angle_cooldown;
  int initTime;
  int tempMaxSpeedInc;
  int tempMaxSpeedDec;
  
  /* status bools */
  bool angleTooHigh;
  bool angleTriggered; 
  bool isInitialized;
  
  /* Ringbuffer */
  RingBufferWithSum<float,100> accX_data;
  RingBufferWithSum<float,5> accX_var_buffer;
  //RingBufferWithSum<float,6000> speeds;
  
  /* Stuff for acceleration*/
  Point accelerate(Point p);
  typedef std::list<std::pair<int,Point> > AccList;
  AccList accBufferX; //FIXME: Value is used uninitialized
  AccList accBufferY; //FIXME: Value is used uninitialized
  AccList accBufferR; //FIXME: Value is used uninitialized
  float getLimitFac(Point &v, float maxAcc, int accDelay, int axis, AccList &accBuffer);
  float getSpdMin(int axis, AccList &accBuffer);
};
