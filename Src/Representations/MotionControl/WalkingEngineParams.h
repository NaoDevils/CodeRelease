/**
* @class WalkingEngineParams
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
* @author <a href="mailto:florian.wilmshoever@tu-dortmund.de">Florian Wilmshöver</a>
*/
#pragma once
#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Angle.h"
#else
#include "bhumanstub.h"
#endif

//#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

/**
* @class WalkingEngineParams
* Contains the parameters of the walking engine. See the "Nao Devils Team Report 2010" for a detailed description.
*/
struct COMOffsets
{
  // translational
  float xFixed; // was xOffset
  float yFixed; // was fixedYOffset
  float xSpeedDependent; // was dynXOffset
  float ySpeedDependent[2]; // 0 = left, 1 = right, was yOffset
  // rotational
  float tiltFixed; // new 25_04_17
  float tiltSpeedDependent[2]; // Min/Max, was speedDependentBodyTilt
  // special case: arm contact
  float xArmContact; // was armContactCoMShiftX
  float yArmContact[2]; // 0 = left, 1 = right, was armContactCoMShift
  
};

struct SpeedLimits
{
  float xForward; // was maxSpeedXForward
  float xForwardArmContact; // was maxSpeedArmContact
  float xForwardOmni; // was maxSpeedXForwardOmni
  float xBackward; // was maxSpeedXBack
  float y; // was maxSpeedY
  float yArmContact; // new 25_04_17 
  Angle r; // was maxSpeedR
  Angle rOnly = 90_deg; // new 07_07_19
};

struct Acceleration
{
  float maxAccXForward; // in m/s
  float maxAccXBackward; // in m/s
  float maxAccY; // in m/s
  float maxAccR; // in rad / s
};

struct SensorControl
{
  unsigned int halSensorDelay; // in frames
  unsigned int sensorDelay; // in frames
  float accXAlpha; // changing comOffsets.xFixed using acc.x sensor
  float sensorControlRatio[2]; // 0 = com, 1 = zmp
  // tilt control
  float tiltControllerParams[3]; // p, i, d
  float tiltFactor; // rotate com around Y: inertialData * tiltFactor
  float steppingRotSpeedUprect;
  float steppingRotSpeedCapture;

  // roll control
  float rollControllerParams[3]; // p, i, d
  float rollFactor; // rotate com around X: inertialData * rollFactor
};

struct FootMovement
{
  // pitch control
  Angle footPitch;
  float footPitchPD[2];
  Angle footRoll;
  float stepHeight[3]; // first: front, second: back, third: sidwards
  float doubleSupportRatio;
  float maxStepDuration;
  float minStepDuration;
  float footYDistance;
  float leadingSideStepSpeedUp; // how much faster the leading step for side steps is. <1 means slower
  float polygonLeft[4];
  float polygonRight[4];
  float forwardPolygon[5];
  float sideStepPolygon[5];
  float rotPolygon[5];
  float heightPolygon[5];
};

struct WalkTransition
{
  int crouchingDownPhaseLength; // in frames
  int startingPhaseLength; // in frames
  int stoppingPhaseLength; // in frames

  float stopPosThresholdX; // in m
  float stopPosThresholdY; // in m
  float stopSpeedThresholdX; // in m/s
  float stopSpeedThresholdY; // in m/s
  Angle fallDownAngleMinMaxX[2]; // in rad
  Angle fallDownAngleMinMaxY[2]; // in rad
  float unstableGyroY; // If gyroY is above this limit the robot is assumed
                       // to be unstable
  int zmpSmoothPhase; // in frames
};

struct WEJointCalibration
{
  union
  {
    struct
    {
      float jointCalibrationLeft[6];
      float jointCalibrationRight[6];
    };
    float jointCalibration[12];
  };
  float offsetLeft[6];
  float offsetRight[6];
  int legJointHardness[6];
};

struct WalkingEngineParams : public Streamable
{
  Acceleration acceleration;
  COMOffsets comOffsets;
  FootMovement footMovement;
  SensorControl sensorControl;
  SpeedLimits speedLimits;
  WalkTransition walkTransition;
  float armFactor;
  float arms1;
  // float maxLegLength; // unused
  int outFilterOrder; // walkingEngineOutput angles are average of ringbuffer (with kinematicoutput) of this size
  
  // Sidestep settings
  float maxSidestep[2];

  WEJointCalibration jointCalibration;

  /** Constructor */
  WalkingEngineParams() {}

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(acceleration.maxAccR)
      STREAM(acceleration.maxAccXForward)
      STREAM(acceleration.maxAccXBackward)
      STREAM(acceleration.maxAccY)
      STREAM(armFactor)
      STREAM(arms1)
      STREAM(comOffsets.tiltFixed)
      STREAM(comOffsets.tiltSpeedDependent)
      STREAM(comOffsets.xArmContact)
      STREAM(comOffsets.xFixed)
      STREAM(comOffsets.xSpeedDependent)
      STREAM(comOffsets.yArmContact)
      STREAM(comOffsets.yFixed)
      STREAM(comOffsets.ySpeedDependent)
      STREAM(footMovement.doubleSupportRatio)
      STREAM(footMovement.footPitch)
      STREAM(footMovement.footPitchPD)
      STREAM(footMovement.footRoll)
      STREAM(footMovement.footYDistance)
      STREAM(footMovement.leadingSideStepSpeedUp)
      STREAM(footMovement.forwardPolygon)
      STREAM(footMovement.sideStepPolygon)
      STREAM(footMovement.rotPolygon)
      STREAM(footMovement.heightPolygon)
      STREAM(footMovement.polygonLeft)
      STREAM(footMovement.polygonRight)
      STREAM(footMovement.maxStepDuration)
      STREAM(footMovement.minStepDuration)
      STREAM(footMovement.stepHeight)
      STREAM(jointCalibration.jointCalibrationLeft)
      STREAM(jointCalibration.jointCalibrationRight)
      STREAM(jointCalibration.legJointHardness)
      STREAM(jointCalibration.offsetLeft)
      STREAM(jointCalibration.offsetRight)
      STREAM(maxSidestep)
      STREAM(outFilterOrder)
      STREAM(sensorControl.accXAlpha)
      STREAM(sensorControl.halSensorDelay)
      STREAM(sensorControl.rollControllerParams)
      STREAM(sensorControl.rollFactor)
      STREAM(sensorControl.sensorControlRatio)
      STREAM(sensorControl.sensorDelay)
      STREAM(sensorControl.steppingRotSpeedCapture)
      STREAM(sensorControl.steppingRotSpeedUprect)
      STREAM(sensorControl.tiltControllerParams)
      STREAM(sensorControl.tiltFactor)
      STREAM(speedLimits.r)
      STREAM(speedLimits.rOnly)
      STREAM(speedLimits.xBackward)
      STREAM(speedLimits.xForward)
      STREAM(speedLimits.xForwardArmContact)
      STREAM(speedLimits.xForwardOmni)
      STREAM(speedLimits.y)
      STREAM(speedLimits.yArmContact)
      STREAM(walkTransition.crouchingDownPhaseLength)
      STREAM(walkTransition.fallDownAngleMinMaxX)
      STREAM(walkTransition.fallDownAngleMinMaxY)
      STREAM(walkTransition.startingPhaseLength)
      STREAM(walkTransition.stoppingPhaseLength)
      STREAM(walkTransition.stopPosThresholdX)
      STREAM(walkTransition.stopPosThresholdY)
      STREAM(walkTransition.stopSpeedThresholdX)
      STREAM(walkTransition.stopSpeedThresholdY)
      STREAM(walkTransition.unstableGyroY)
      STREAM(walkTransition.zmpSmoothPhase)
      STREAM_REGISTER_FINISH;
  };
  /** Descructor */
  ~WalkingEngineParams()
  {};

};

STREAMABLE(JointControlParameters,
{,
  (float)(1.f) pidMultiplicator,
  (Angle)(0_deg) deltaAngleX,
  (Angle)(0_deg) deltaAngleBack,
  (Angle)(0_deg) deltaAngleFront,
  (float)(0.f) p_x,
  (float)(0.f) i_x,
  (float)(0.f) d_x,
  (float)(0.f) p_y,
  (float)(0.f) i_y,
  (float)(0.f) d_y,
  (float)(0.f) comX_p,
  (float)(0.8f) angleGyroRatioX,
  (float)(0.9f) angleGyroRatioY,
});

STREAMABLE(BalanceParameters,
{ ,
  (Angle)(0_deg) targetAngleX,
  (Angle)(2_deg) targetAngleY,
  (JointControlParameters) ankleParams,
  (JointControlParameters) hipParams,
});

STREAMABLE(LegJointSensorControlParameters,
{,
  (BalanceParameters) walkBalanceParams,
  (BalanceParameters) specialActionBalanceParams,
});

STREAMABLE(COMShiftParameters,
{,
  (float)(0.f) accXAlpha,
  (float)(0.f) gyroYAlpha,
  (float)(0.f) angleYAlpha,
  (float)(0.f) gyroXAlpha,
  (float)(0.f) angleXAlpha,
  (float)(0.f) stepAccAlpha,
  (int)(200) accInterpolTime,
});

//struct FreeLegPhaseParams : public WalkingEngineParams { };

