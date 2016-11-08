/**
* @class WalkingEngineParams 
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
* @author <a href="mailto:florian.wilmshoever@tu-dortmund.de">Florian Wilmshöver</a>
*/
#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#else
#include "bhumanstub.h"
#endif

//#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

#ifndef LEFT_FOOT
#define LEFT_FOOT 0
#endif

#ifndef RIGHT_FOOT
#define RIGHT_FOOT 1
#endif

/**
* @class WalkingEngineParams 
* Contains the parameters of the walking engine. See the "Nao Devils Team Report 2010" for a detailed description.
*/
struct WalkingEngineParams : public Streamable
{
  float footPitch;
  float footPitchPD[2];
  float maxWalkPitch;
  float pitchSpeed;
  float xOffset;
  float stepHeight[2]; // first: normal, second: value for max y speed
  float sensorControlRatio[2];
  float doubleSupportRatio;
  int crouchingDownPhaseLength;
  int startingPhaseLength;
  int stoppingPhaseLength;
  float armFactor;
  float arms1;
  int zmpSmoothPhase;
  int maxSpeedXForward;
  int maxSpeedXForwardOmni;
  int maxSpeedXBack;
  int maxSpeedY;
  float maxSpeedR;
  float maxStandPitch;
  float maxLegLength;
  float stepDuration;
  float footYDistance;
  float stopPosThresholdX;
  float stopPosThresholdY;
  float stopSpeedThresholdX;
  float stopSpeedThresholdY;
  float zmpLeftX;
  float zmpLeftY;
  float zmpRightX;
  float zmpRightY;
  int outFilterOrder;
  float tiltFactor;
  float rollFactor;
  float tiltControllerParams[3];
  float rollControllerParams[3];
  unsigned int sensorDelay;
  unsigned int halSensorDelay;
  float maxFootSpeed;
  float fallDownAngleMinMaxX[2];
  float fallDownAngleMinMaxY[2];
  float polygonLeft[4];
  float polygonRight[4];
  float offsetLeft[6];
  float offsetRight[6];
  int walkRequestFilterLen;
  float accXAlpha;
  float maxAccX;
  int accDelayX;
  float maxAccY;
  int accDelayY;
  float maxAccR;
  int accDelayR;
  int speedApplyDelay;
  //hardness settings
  int legJointHardness[6];
  float heightPolygon[5];
  // kick settings
  float kickStart[2];
  float kickStop[2];
  float angleTiltP;
  float ballXMin[2];
  float ballXMax[2];
	float ballYMin[2];  // index 0 for a kick with direction 0,
                      // index 1 for a kick with max direction
	float ballYMax[2];  // same here
  float rotMin;
  float rotMax; 
  float zmpMax;
  float zmpMoveSpeedY;
  float standRollFactor;
  float CoMZDiff;
  float stepOffsetFactor;
  // Sidestep settings
  float maxSidestep[2];
  float minSidestep[2];
  float forwardPolygon[5];
  int kickStartLen[2];
  int kickStopLen[2];
  float fixedBallPos[2];
  union WEJointCalibration
  {
    struct 
    {
      float jointCalibrationLeft[6];
      float jointCalibrationRight[6];
    };
    float jointCalibration[12];
  };
  WEJointCalibration jointCalibration;
  float footRoll;
  float speedDependentBodyTilt[2]; // Min/Max
  float yOffset[2]; // Depends on y speed! Index 0 is left, 1 is right
  float fixedYOffset; // This one applies always.
  float dynXOffset;
  
  /** Constructor */
  WalkingEngineParams() {}

  void serialize(In* in,Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(accDelayR)
    STREAM(accDelayX)
    STREAM(accDelayY)
    STREAM(accXAlpha)
    STREAM(angleTiltP)
    STREAM(armFactor)
    STREAM(arms1)
    STREAM(ballXMax)
    STREAM(ballXMin)
    STREAM(ballYMax)
    STREAM(ballYMin)
    STREAM(CoMZDiff)
    STREAM(crouchingDownPhaseLength)
    STREAM(doubleSupportRatio)
    STREAM(dynXOffset)
	  STREAM(fallDownAngleMinMaxX)
	  STREAM(fallDownAngleMinMaxY)
    STREAM(fixedBallPos)
    STREAM(fixedYOffset)
    STREAM(footPitch)
    STREAM(footPitchPD)
    STREAM(footRoll)
    STREAM(footYDistance)
    STREAM(forwardPolygon)
    STREAM(halSensorDelay)
    STREAM(heightPolygon)
    STREAM(jointCalibration.jointCalibrationLeft)
    STREAM(jointCalibration.jointCalibrationRight)
    STREAM(kickStart)
    STREAM(kickStartLen)
    STREAM(kickStop)
    STREAM(kickStopLen)
    STREAM(legJointHardness)
    STREAM(maxAccR)
    STREAM(maxAccX)
    STREAM(maxAccY)
    STREAM(maxFootSpeed)
    STREAM(maxLegLength)
    STREAM(maxSidestep)
    STREAM(maxSpeedR)
    STREAM(maxSpeedXBack)
    STREAM(maxSpeedXForward)
    STREAM(maxSpeedXForwardOmni)
    STREAM(maxSpeedY)
    STREAM(maxStandPitch)
    STREAM(maxWalkPitch)
    STREAM(minSidestep)
    STREAM(offsetLeft)
    STREAM(offsetRight)
    STREAM(outFilterOrder)
    STREAM(pitchSpeed)
    STREAM(polygonLeft)
    STREAM(polygonRight)
    STREAM(rollControllerParams)
    STREAM(rollFactor)
    STREAM(rotMax)
    STREAM(rotMin)
    STREAM(sensorControlRatio)
    STREAM(sensorDelay)
    STREAM(speedApplyDelay)
    STREAM(speedDependentBodyTilt)
    STREAM(standRollFactor)
    STREAM(startingPhaseLength)
    STREAM(stepDuration)
    STREAM(stepHeight)
    STREAM(stepOffsetFactor)
    STREAM(stoppingPhaseLength)
    STREAM(stopPosThresholdX)
    STREAM(stopPosThresholdY)
    STREAM(stopSpeedThresholdX)
    STREAM(stopSpeedThresholdY)
    STREAM(tiltControllerParams)
    STREAM(tiltFactor)
    STREAM(walkRequestFilterLen)
    STREAM(xOffset)
    STREAM(yOffset)
    STREAM(zmpLeftX)
    STREAM(zmpLeftY)
    STREAM(zmpMax)
    STREAM(zmpMoveSpeedY)
    STREAM(zmpRightX)
    STREAM(zmpRightY)
    STREAM(zmpSmoothPhase)
    STREAM_REGISTER_FINISH;
  };
  /** Descructor */
  ~WalkingEngineParams()
  {};

};

struct FreeLegPhaseParams : public WalkingEngineParams { };

