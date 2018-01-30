/**
 * @file Modules/MotionControl/DortmundWalkingEngine/RequestTranslator.cpp
 * Translates the MotionRequest for the WalkingEngine (some clipping and path-calculations)
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
 * @author <a href="mailto:stefan.czarnetzki@uni-dortmund.de">Stefan Czarnetzki</a>
 */

#include "RequestTranslator.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"
#include <string>
#include "Modules/MotionControl/MotionSelector.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Geometry.h"
#include <utility>
#include <cfloat>
//#define LOGGING
//#include "Tools/Debugging/CSVLogger.h"

using namespace std;

RequestTranslator::RequestTranslator(const MotionSelection &theMotionSelection,
                                     const WalkingInfo &theWalkingInfo,
                                     const FallDownState &theFallDownState,
                                     const InertialSensorData & theInertialSensorData,
                                     const BallModelAfterPreview &theBallModelAfterPreview,
									 const SpeedInfo &theSpeedInfo,
                   const SpeedRequest &theSpeedRequest,
                                     const ArmMovement &theArmMovement,
  const GoalSymbols &theGoalSymbols
									 ):
theMotionSelection(theMotionSelection),
theWalkingInfo(theWalkingInfo),
theFallDownState(theFallDownState),
theInertialSensorData(theInertialSensorData),
theBallModelAfterPreview(theBallModelAfterPreview),
theSpeedInfo(theSpeedInfo),
theSpeedRequest(theSpeedRequest),
theArmMovement(theArmMovement),
theGoalSymbols(theGoalSymbols),
timer(300)
{

  InMapFile stream("modules.cfg");
  if (stream.exists())
  {
    stream >> config;
  }
  else
  {
    ASSERT(false);
  }

  bool flipm = false;
  for (ModuleManager::Configuration::RepresentationProvider por : config.representationProviders) {
    if (por.representation == "TargetCoM" && por.provider == "FLIPMController") {
      flipm = true;
      break;
    }
  }

  if (flipm) {
    InMapFile file("walkingParamsFLIPM.cfg");
    if (file.exists())
      file >> walkParams;
    else
    {
      ASSERT(false);
    }
  } else {
    InMapFile file("walkingParams.cfg");
    if (file.exists())
      file >> walkParams;
    else
    {
      ASSERT(false);
    }
  }

  lastN=0;
 
  walkStarted = false;
  
  for (int i=0; i<3; i++)
    filter[i].createBuffer(walkParams.acceleration.walkRequestFilterLen);
  
  ASSERT(walkParams.acceleration.accDelayX <= MAX_ACC_WINDOW);
  ASSERT(walkParams.acceleration.accDelayY <= MAX_ACC_WINDOW);
  ASSERT(walkParams.acceleration.accDelayR <= MAX_ACC_WINDOW);

  /* init data for accX-criterion*/
  tempMaxSpeed = 100;
  theFactor = 0;

  /* init status bools */
  isInitialized = false;
  angleTooHigh = false;
  angleTriggered = false;

  /* init 'timer' to 0 */
  angle_cooldown = 0;
  initTime = 0;
  tempMaxSpeedInc = 0;
  tempMaxSpeedDec = 0;

  reset();
}

PatternGenRequest::State RequestTranslator::getNewState(float x, float y, float r)
{
  if (currentState==PatternGenRequest::emergencyStop)
    return PatternGenRequest::standby;  // Try to restart engine (maybe interrupted if robot
  // is still not upright
  
  if (!(theMotionSelection.targetMotion==MotionRequest::walk &&
    theMotionSelection.ratios[MotionRequest::walk] == 1.f) &&
      (currentState==PatternGenRequest::ready || currentState==PatternGenRequest::standby))
    return PatternGenRequest::standby;
  
  if (!(theMotionSelection.targetMotion==MotionRequest::walk &&
    theMotionSelection.ratios[MotionRequest::walk] == 1.f) &&
      currentState==PatternGenRequest::walking)
    return PatternGenRequest::ready;
  
  if (theMotionSelection.targetMotion==MotionRequest::walk &&
    theMotionSelection.ratios[MotionRequest::walk] == 1.f)
  {
    if (std::abs(x)>0.001 ||
        std::abs(y)>0.001 ||
        std::abs(r)>0.01)
      return PatternGenRequest::walking;
    else
      return PatternGenRequest::ready;
  }
  
  return PatternGenRequest::NA;
}


RequestTranslator::~RequestTranslator(void)
{
}

void RequestTranslator::reset()
{
  Point nullPoint;
  accBufferX.clear();
  accBufferX.push_back(std::pair<int, Point>(0, nullPoint));
  accBufferY.clear();
  accBufferY.push_back(std::pair<int, Point>(0, nullPoint));
  accBufferR.clear();
  accBufferR.push_back(std::pair<int, Point>(0, nullPoint));
}

Pose2f RequestTranslator::gotoPointController(const Pose2f &target, const Pose2f &max)
{
  float vx = paramsGoToRequest.kpx * target.translation.x();
  if (vx > max.translation.x()) vx = max.translation.x();
 // if (std::abs(target.translation.x()) < 20) vx = 150;
  
  float vy = paramsGoToRequest.kpy * target.translation.y();
  if (vy > max.translation.y()) vy = max.translation.y();
  if (vy < -max.translation.y()) vy = -max.translation.y();
  
 // if (std::abs(target.translation.y()) < 20) vy = 150;
  
  int distFac = ((int)target.translation.x() - 50);
  if (distFac < 1) distFac = 1;
  distFac = 6 - (distFac + 5) / distFac;
  float vr = paramsGoToRequest.kpr * target.rotation * (distFac / (distFac + 1.f));
  if (std::abs(target.rotation) < 0.1f) vr = 0;
  
  Pose2f motionRequest;
  motionRequest.translation.x() = vx;
  motionRequest.translation.y() = vy;
  motionRequest.rotation = vr;
  return motionRequest;
  
}

Pose2f RequestTranslator::gotoPointController(const Vector2f &target, const Pose2f &max)
{
  //adjust rotation so when the robot is close to the target the rotation is lowered
  float rot = (float)atan2(target.y(), target.x());
  float dist = (float)sqrt(target.y()*target.y() + target.x()*target.x());
  float REAC_DIST = 100.f;
  if(dist < REAC_DIST)
  {
    //if closer than 10cm (REAC_DIST) to ball, adjust rotation
    float fac = sqr(dist/REAC_DIST);
    if(fac < 0.3f)
    {
      //clip the factor at 0.3
      fac = 0.3f;
    }
    rot *= fac;
  }
  return gotoPointController(Pose2f(rot, target), max);
}

Pose2f RequestTranslator::gotoBallAndStand()
{
  Vector2f target(theBallModelAfterPreview.estimate.position.x() - theMotionSelection.walkRequest.request.translation.x(),
                           theBallModelAfterPreview.estimate.position.y() + theMotionSelection.walkRequest.request.translation.y());
  return gotoPointController(target,
    Pose2f(walkParams.speedLimits.r, walkParams.speedLimits.xForward, walkParams.speedLimits.y));
}

Pose2f RequestTranslator::gotoBallAndKick()
{
  Vector2f target(theBallModelAfterPreview.estimate.position.x() - theMotionSelection.walkRequest.request.translation.x(),
                           theBallModelAfterPreview.estimate.position.y() + theMotionSelection.walkRequest.request.translation.y());
  return gotoPointController(target,
    Pose2f(walkParams.speedLimits.r, walkParams.speedLimits.xForward, walkParams.speedLimits.y));
}

/*
  Some notes regarding acceleration.
  It is not possible to limit deceleration while 
  maintaining the speed vector direction. E.g.
  you want from 200/0/1 to 300/0/0, a fast curve
  to straight forward, but you must decelerate to
  0.5 first. It is not possible in this situation
  to maintain direction straight forward as you
  would need to walk with endless speed forward.
  This problem is not symmetric. Acceleration
  is possible while maintaining direction as
  walking endless fast is not possible but
  walking endless slow IS possible (speed 0).
  So limiting all directions is ever possible.
  You could maintain the direction of the current
  vector in case of deceleration, that would be
  the symmetric case, but we don't want this.

  Another problem, if we must limit the acc of an
  axis, can we dec other axis to maintain direction?
  Or are then the dec max of the other axis a problem?

  So, deceleration is never limited.
*/

float RequestTranslator::getSpdMin(int axis, AccList &accBuffer)
{
  float min = FLT_MAX;
  for (AccList::iterator a = accBuffer.begin(); a != accBuffer.end(); a++)
  {
    if (std::abs(a->second.v[axis]) < std::abs(min))
      min = (float)a->second.v[axis];
  }
  return min;
}

float RequestTranslator::getLimitFac(Point &v, float maxAcc, int accDelay, int axis, AccList &accBuffer)
{
  if (accBuffer.size() > 1 && accBuffer.front().first >= accBuffer.back().first)
    reset();
  while (accBuffer.front().first + accDelay < accBuffer.back().first)
    accBuffer.pop_front();
  accBuffer.push_back(std::pair<int, Point>(theSpeedInfo.timestamp, Point(theSpeedInfo.speedBeforePreview)));

  // Maybe also check if the sign of the speeds changes (if one speed is != 0), and if yes just return 0
  
  float last = getSpdMin(axis, accBuffer);
  float desired = (float)v.v[axis];
  float diff = desired - last;

  if (std::abs(desired) <= std::abs(last) ||  std::abs(desired) - std::abs(last) < maxAcc)
    return 1.f;

  /* We must accelerate when a higher speed is desired,
     which is especially the case when the sign changes.
   */
  if (sgn(desired) != sgn(last))
  {
    last = 0;
  }

  /* Now the speed we can achieve with given limits */
  float targetSpeed = last + sgn(diff) * maxAcc;

  /* Now we can return the factor f for targetSpeed = f * desired */
  return targetSpeed / desired;
}

Point RequestTranslator::accelerate(Point p)
{
  // TODO auslagern in Parameter
  float fx;
  if (theSpeedInfo.speed.translation.x() > 0.02) {
    fx = getLimitFac(p, walkParams.acceleration.maxAccX, walkParams.acceleration.accDelayX, 0, accBufferX);
  }
  else {
    fx = getLimitFac(p, (walkParams.acceleration.maxAccX/2.f), walkParams.acceleration.accDelayX, 0, accBufferX);
  }

  float fy = getLimitFac(p, walkParams.acceleration.maxAccY, walkParams.acceleration.accDelayY, 1, accBufferY);
  float fr = getLimitFac(p, walkParams.acceleration.maxAccR, walkParams.acceleration.accDelayR, 5, accBufferR);

  float f = std::min(std::min(fx,fy),fr);

  PLOT("module:RequestTranslator:fx",fx);
  PLOT("module:RequestTranslator:fy",fy);
  PLOT("module:RequestTranslator:fr",fr);
  PLOT("module:RequestTranslator:f",f);

  p *= f;
  return p;
}

void RequestTranslator::updatePatternGenRequest(PatternGenRequest & patternGenRequest)
{
  DECLARE_DEBUG_DRAWING( "module:RequestTranslator:Path", "drawingOnField"); // should be displayed relative to RobotPoseAfterPreview
  DECLARE_DEBUG_DRAWING( "module:RequestTranslator:HowTranslated", "drawingOnField");
  DECLARE_DEBUG_DRAWING( "module:RequestTranslator:TranslationResultAsSpeed", "drawingOnField");
  Point offset(-200, 0);
  offset.rotate2D(theGoalSymbols.centerAngleBallToOppGoalWC);
  Vector2f target = theBallModelAfterPreview.estimate.position + Vector2f(offset.x, offset.y);
  Pose2f max(1, 250, 200);
  switch (theMotionSelection.walkRequest.requestType)
  {
    case WalkRequest::speed:
    {
      // This will be executed if the behavior requests a walking speed directly:
      patternGenRequest.speed.translation.x()=theMotionSelection.walkRequest.request.translation.x();
      patternGenRequest.speed.translation.y()=theMotionSelection.walkRequest.request.translation.y();
      patternGenRequest.speed.rotation=theMotionSelection.walkRequest.request.rotation;
    }
      break;
    case WalkRequest::destination:
    {
      // This will be executed if the behavior requests a goto target:
      
      patternGenRequest.speed.translation = theSpeedRequest.translation;
      patternGenRequest.speed.rotation = theSpeedRequest.rotation;
    }
      break;
    case WalkRequest::ball:
    {
      // This will be executed when moving directly to the ball and ignoring any orientation issues..
      
      if (theMotionSelection.walkRequest.kickStrength > 0)
        patternGenRequest.speed = gotoBallAndKick();
      else
        patternGenRequest.speed = gotoBallAndStand();
    }
      break;
    case WalkRequest::dribble:
       patternGenRequest.speed = gotoPointController(target, max);
      break;
    default:
      // TODO: do error handling here
      break;
  }
  patternGenRequest.pitch=0;

  PLOT("module:RequestTranslator:speed_before_clipping.x",patternGenRequest.speed.translation.x());
  PLOT("module:RequestTranslator:speed_before_clipping.y",patternGenRequest.speed.translation.y());
  PLOT("module:RequestTranslator:speed_before_clipping.r",patternGenRequest.speed.rotation);

  clipping(patternGenRequest.speed);
   
  patternGenRequest.speed.translation.x()/=1000;
  patternGenRequest.speed.translation.y()/=1000;

  patternGenRequest.speed = accelerate(patternGenRequest.speed);

  /*LOG("MotionRequests", "x",patternGenRequest.speed.translation.x);
  LOG("MotionRequests", "y",patternGenRequest.speed.translation.y);
  LOG("MotionRequests", "r",patternGenRequest.speed.rotation);*/
  
  
  patternGenRequest.speed.translation.x()=filter[0].nextValue(patternGenRequest.speed.translation.x());
  patternGenRequest.speed.translation.y()=filter[1].nextValue(patternGenRequest.speed.translation.y());
  patternGenRequest.speed.rotation=filter[2].nextValue(patternGenRequest.speed.rotation);
  
  patternGenRequest.newState=getNewState(patternGenRequest.speed.translation.x(), patternGenRequest.speed.translation.y(), patternGenRequest.speed.rotation);
  if (currentState == PatternGenRequest::standby && patternGenRequest.newState == PatternGenRequest::ready)
    timer = 300; // Reset stand timer if we come from another motion engine
  currentState=patternGenRequest.newState;
  
  //if (lastN!=contwalkParams.N)
  //speedBuffer.init(contwalkParams.N);
  lastN=contParams.N;
  
  PLOT("module:RequestTranslator:speed_after_clipping.x",patternGenRequest.speed.translation.x() * 1000);
  PLOT("module:RequestTranslator:speed_after_clipping.y",patternGenRequest.speed.translation.y() * 1000);
  PLOT("module:RequestTranslator:speed_after_clipping.r",patternGenRequest.speed.rotation);
  
    
  PLOT("module:RequestTranslator:speedX",patternGenRequest.speed.translation.x());
  PLOT("module:RequestTranslator:speedY",patternGenRequest.speed.translation.y());

  paramsGoToRequest.handle();
  paramswalkingDecelerate.handle();
}

void RequestTranslator::clipping(Pose2f &speed)
{  
  // Check for instabilities and adapt walking speed
  deceleratedByInstability = false;
  deceleratedByMax = false;
  
  //fill accX_data buffer with sensor data, if in range (-10,10);
  //else fill with copy of last data
  if(theFallDownState.state==FallDownState::upright){
	if(std::abs(theInertialSensorData.acc.x()* -0.01558f) < 10){
		accX_data.push_front(theInertialSensorData.acc.x()* -0.01558f);
	} else{
		accX_data.push_front(accX_data[0]);
	}
  }
  //calculate variance and add it to buffer for theFactor
  float accX_var = accX_data.getVariance();
  accX_var_buffer.push_front(accX_var);

  PLOT("module:RequestTranslator:accX_var", accX_var);
  PLOT("module:RequestTranslator:accX_var_border", paramswalkingDecelerate.var_border);

  PLOT("module:RequestTranslator:inc_timer", tempMaxSpeedInc);
  PLOT("module:RequestTranslator:dec_timer", tempMaxSpeedDec);

  PLOT("module:RequestTranslator:angleX", std::abs(theInertialSensorData.angle.x()));
  PLOT("module:RequestTranslator:angleY", std::abs(theInertialSensorData.angle.y()));
  PLOT("module:RequestTranslator:angle_border", paramswalkingDecelerate.angle_border);

  PLOT("module:RequestTranslator:factor",theFactor);
  float angleX = theInertialSensorData.angle.x();
  float angleY =theInertialSensorData.angle.y();
  if (angleX < walkParams.walkTransition.fallDownAngleMinMaxX[0] ||
      angleX > walkParams.walkTransition.fallDownAngleMinMaxX[1] ||
	    angleY < walkParams.walkTransition.fallDownAngleMinMaxY[0] ||
      angleY > walkParams.walkTransition.fallDownAngleMinMaxY[1] ||
      theFallDownState.state!=FallDownState::upright)
    timer = 0;

  if ((unsigned int)timer <= paramswalkingDecelerate.standTime)
	{
    MotionSelector::stand();
    deceleratedByInstability = true;
    timer++;
  }
  else if (timer == paramswalkingDecelerate.standTime)
  {
    MotionSelector::stand();
    timer ++;
  }
  else if (timer >= paramswalkingDecelerate.standTime)
  {
     MotionSelector::move();
  }

  
  if (speed.translation.x()>walkParams.speedLimits.xForward)
  {
    speed.translation.x() = walkParams.speedLimits.xForward;
    deceleratedByMax = true;
  }
  
  if (speed.translation.x()<-walkParams.speedLimits.xBackward)
  {
    speed.translation.x() = (-walkParams.speedLimits.xBackward);
    deceleratedByMax = true;
  }
  
  if (speed.translation.y()>walkParams.speedLimits.y)
  {
    speed.translation.y() = walkParams.speedLimits.y;
    deceleratedByMax = true;
  }
  
  if (speed.translation.y()<-walkParams.speedLimits.y)
  {
    speed.translation.y() = (-walkParams.speedLimits.y);
    deceleratedByMax = true;
  }
  
  if (speed.rotation<-walkParams.speedLimits.r)
  {
    speed.rotation=-walkParams.speedLimits.r;
    deceleratedByMax = true;
  }
  
  if (speed.rotation>walkParams.speedLimits.r)
  {
    speed.rotation= walkParams.speedLimits.r;
    deceleratedByMax = true;
  }
  
  if (speed.translation.x() > walkParams.speedLimits.xForwardOmni && speed.rotation > walkParams.speedLimits.r / 2)
  {
    speed.translation.x() = walkParams.speedLimits.xForwardOmni;
    speed.rotation = walkParams.speedLimits.r / 2;
  }
  
  if (speed.translation.x() > walkParams.speedLimits.xForwardOmni && speed.rotation < -walkParams.speedLimits.r / 2)
  {
    speed.translation.x() = walkParams.speedLimits.xForwardOmni;
    speed.rotation = -walkParams.speedLimits.r / 2;
  }
  
  if (speed.translation.x() > walkParams.speedLimits.xForwardOmni && speed.translation.y() > walkParams.speedLimits.y / 2)
  {
    speed.translation.x() = walkParams.speedLimits.xForwardOmni;
    speed.translation.y() = walkParams.speedLimits.y / 2;
  }
  
  if (speed.translation.x() > walkParams.speedLimits.xForwardOmni && speed.translation.y() < -walkParams.speedLimits.y / 2)
  {
    speed.translation.x() = walkParams.speedLimits.xForwardOmni;
    speed.translation.y() = -walkParams.speedLimits.y / 2;
  }
  
  if (theArmMovement.armsInContactAvoidance && speed.translation.x() > walkParams.speedLimits.xForwardArmContact)
  {
    speed.translation.x() = walkParams.speedLimits.xForwardArmContact;
  }
  
  float speedAbsY = std::abs(speed.translation.y());
  float speedAbsX = std::abs(speed.translation.x());
  float speedSum = speedAbsX+speedAbsY;
  if (speedAbsX > paramsGoToRequest.minSpeedForUsingSpeedSum
    && speedAbsY > paramsGoToRequest.minSpeedForUsingSpeedSum
    && speedSum > paramsGoToRequest.maxSpeedSum)
  {
    speed.translation *= paramsGoToRequest.maxSpeedSum/speedSum;
    deceleratedByMax = true;
  }
  
  if (speed.translation.y()>walkParams.speedLimits.y)
  {
    speed.translation.y() = walkParams.speedLimits.y;
    deceleratedByMax = true;
  }
  
  if (speed.translation.y()<-walkParams.speedLimits.y)
  {
    speed.translation.y() = -walkParams.speedLimits.y;
    deceleratedByMax = true;
  }
 
  if(!paramswalkingDecelerate.disableDynamicSpeed){
    if(!isInitialized){
	  //Nao needs to be walking to get valid data
      if(speed.translation.x() != 0){
        //wait totalInitTime to have filled accX_data buffer with valid data
        if(initTime++ == paramswalkingDecelerate.totalInitTime){
          isInitialized = true;
        }
      }
    } else{
	  //if maxSpeed was reduced by angle, use a cooldown to prevent instant timer toggle
	  if(angle_cooldown > 0) angle_cooldown--;
	  //check if angle is too high and set angleTriggered to true if it wasnt too high in last frame
      if(std::abs(theInertialSensorData.angle.x()) > paramswalkingDecelerate.angle_border || std::abs(theInertialSensorData.angle.y()) > paramswalkingDecelerate.angle_border)
      {
        if(angle_cooldown == 0 && angleTooHigh == false)
        {
          angleTriggered = true;
		      angle_cooldown = 150;
        }
        angleTooHigh = true;
      }
      else
        angleTooHigh = false;

	  //if Nao not upright => reset timer and set tempMaxSpeed to starting value (100)
	  if(theFallDownState.state!=FallDownState::upright){
		  tempMaxSpeedInc = 0;
		  tempMaxSpeedDec = 0;
		  tempMaxSpeed = 100;
	  }
	
	  //calculate factor for the speedchange
      theFactor = paramswalkingDecelerate.var_border - accX_var_buffer.average();
	  //if Factor positive, multiply with fPos, otherwise with fNeg
	  theFactor *= (theFactor > 0) ? paramswalkingDecelerate.fPos : paramswalkingDecelerate.fNeg;

      // Detect bad walk and slow nao down/speed up if not bad anymore
      if((accX_var >= paramswalkingDecelerate.var_border || angleTooHigh) && theFallDownState.state==FallDownState::upright){
		//increase timer if cooldowned
        if(angle_cooldown == 0) tempMaxSpeedDec++;
        tempMaxSpeedInc = 0;
		//if timer reached its max or angle was triggered, decrease the speed by either
        if(tempMaxSpeedDec == 100 || angleTriggered){
		  //if the angle was triggered, decrease tempMaxSpeed by angleSpeedchange
          if(angleTriggered){ 
            angleTriggered = false;
            if(tempMaxSpeed + paramswalkingDecelerate.angleSpeedchange >= 0){
              tempMaxSpeed += paramswalkingDecelerate.angleSpeedchange;	//note: angleSpeedchange must be negative
            }
		  //if the decrease timer triggered the decrease, 
		  //decrease the speed by dynamic speedchange using theFactor if walking faster than 0.
          }else if(tempMaxSpeedDec == 100){  
			float speedchange = (theFactor * 50);
			//Sometimes speedchange might be positive, then use -10 instead.
            if(speedchange > 0){
			  speedchange = -10;
			}
            if((speedchange < 0) && (tempMaxSpeed + speedchange >= 0)){ 
              tempMaxSpeed += speedchange;
            }
          }
		  //reset timer after decreasing tempMaxSpeed
          tempMaxSpeedDec = 0;
        }
      } else{
        //if criterion is not triggered: increase the increase timer
        if(speed.translation.x() >= tempMaxSpeed){ 
          tempMaxSpeedInc++;
        }
        tempMaxSpeedDec = 0;
		//if timer reached 250, increase the speed using theFactor
        if(tempMaxSpeedInc == 250){
          tempMaxSpeedInc = 0;
          float speedchange = (theFactor * 50);
		  //Increasing with negative speed results in a decrease, so use 5 instead.
		  if(speedchange < 0){
			  speedchange = 5;
          }
		  //inrease the speedchange if result is not faster than maxSpeedXForward, else use maxSpeedXForward as tempMaxSpeed
          if(tempMaxSpeed + speedchange <= walkParams.speedLimits.xForward){
            tempMaxSpeed += speedchange;
          } else if(tempMaxSpeed < walkParams.speedLimits.xForward){
            tempMaxSpeed = walkParams.speedLimits.xForward;
          }
        }
      }
      //Clip speed to tempMaxSpeed
      if(speed.translation.x() >= tempMaxSpeed){
        speed.translation.x() = tempMaxSpeed;
        deceleratedByInstability = true;
      }
    }
  }
}

void RequestTranslator::updateWalkingEngineParams(WalkingEngineParams & walkingEngineParams)
{
  MODIFY("representation:WalkingEngineParams", walkParams);
  walkingEngineParams=walkParams;
}

void RequestTranslator::updateFLIPMObserverParams(FLIPMObserverParams & flipmObserverParams)
{
  MODIFY("representation:FLIPMObserverParams", observerParams);
  flipmObserverParams = observerParams;
}

