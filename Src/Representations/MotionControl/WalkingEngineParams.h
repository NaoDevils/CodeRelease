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

#define MAX_DELAY_FRAMES 6

/**
* @class WalkingEngineParams
* Contains the parameters of the walking engine. See the "Nao Devils Team Report 2010" for a detailed description.
*/

STREAMABLE(Acceleration,,
  (Angle)(60_deg) maxAccR, // in rad / s^2
  (float)(0.15f) maxAccXForward, // in m/s^2
  (float)(0.08f) maxAccXBackward, // in m/s^2
  (float)(0.15f) maxAccY // in m/s^2
);

STREAMABLE(COMOffsets,,
  (Angle)(0_deg) tiltFixed, // new 25_04_17
  (Angle[2]) tiltSpeedDependent, // Min/Max, was speedDependentBodyTilt
  (float)(0.f) xArmContact, // was armContactCoMShiftX
  (float)(0.f) xFixed, // was xOffset
  (float)(0.f) xSpeedDependent, // was dynXOffset
  (float[2]) yArmContact, // 0 = left, 1 = right, was armContactCoMShift
  (float)(0.f) yFixed, // was fixedYOffset
  (float[2]) ySpeedDependent // 0 = left, 1 = right, was yOffset
);

STREAMABLE(FootMovement,,
  (float)(0.f) doubleSupportRatio,
  (Angle)(0_deg) footPitch,
  (Angle)(0_deg) footRoll,
  // pitch control
  (float[2]) footPitchPD,
  (float)(0.f) footYDistance,
  (float)(0.f) leadingSideStepSpeedUp, // how much faster the leading step for side steps is. <1 means slower
  (float[5]) forwardPolygon,
  (float[5]) sideStepPolygon,
  (float[5]) rotPolygon,
  (float[5]) heightPolygon,
  (float[4]) polygonLeft,
  (float[4]) polygonRight,
  (float)(0.f) maxStepDuration,
  (float)(0.f) minStepDuration,
  (float[3]) stepHeight // first: front, second: back, third: sidwards
);

STREAMABLE(WEJointCalibration,
  Angle * jointCalibration = jointCalibrationLeft;
  ,                                 // HipYawPitch,
  (Angle[6]) jointCalibrationLeft,  // HipRoll,
  (Angle[6]) jointCalibrationRight, // HipPitch,
  (int[6]) legJointHardness,        // KneePitch,
  (float[6]) offsetLeft,            // AnklePitch,
  (float[6]) offsetRight            // AnkleRoll
);

STREAMABLE(SpeedLimits,,
  (float)(0.f) speedFactor,
  (Angle)(0_deg) r, // was maxSpeedR
  (Angle)(90_deg) rOnly, // new 07_07_19
  (float)(0.f) xBackward, // was maxSpeedXBack
  (float)(0.f) xForward, // was maxSpeedXForward
  (float)(0.f) xForwardArmContact, // was maxSpeedArmContact
  (float)(0.f) xForwardOmni, // was maxSpeedXForwardOmni
  (float)(0.f) y, // was maxSpeedY
  (float)(0.f) yArmContact // new 25_04_17 
);

STREAMABLE(WalkTransition,,
  (int)(0) crouchingDownPhaseLength, // in frames
  (Angle[2]) fallDownAngleMinMaxX, // in rad
  (Angle[2]) fallDownAngleMinMaxY, // in rad
  (float)(0.f) stopSpeedThresholdX, // in m/s
  (float)(0.f) stopSpeedThresholdY, // in m/s
  (Angle)(24_deg) fallDownAngleFront,
  (Angle)(24_deg) fallDownAngleSide,
  (Angle)(-17_deg) fallDownAngleBack
);

STREAMABLE(WalkingEngineParams,,
  (Acceleration) acceleration,
  (COMOffsets) comOffsets,
  (FootMovement) footMovement,
  (WEJointCalibration) jointCalibration,
  (unsigned int)(5) jointSensorDelayFrames, // in frames
  (unsigned int)(3) imuSensorDelayFrames, // in frames
  (int)(5) outFilterOrder, // walkingEngineOutput angles are average of ringbuffer (with kinematicoutput) of this size
  (SpeedLimits) speedLimits,
  (WalkTransition) walkTransition
  // (float)(0.f) maxLegLength // unused
);

STREAMABLE(JointControlParameters,,
  (float)(0.f) zeroPidMultiplicatorX,
  (float)(1.f) pidMultiplicatorX,
  (float)(0.f) p_x,
  (float)(0.f) i_x,
  (float)(0.f) d_x,
  (float)(0.f) zeroPidMultiplicatorY,
  (float)(1.f) pidMultiplicatorY,
  (float)(0.f) p_y,
  (float)(0.f) i_y,
  (float)(0.f) d_y,
  (float)(0.f) comX_p
);

STREAMABLE(BalanceParameters,,
  (Angle)(0_deg) targetAngleX,
  (Angle)(2_deg) targetAngleY,
  (JointControlParameters) ankleParams,
  (JointControlParameters) hipParams,
  (float)(0.7f) ankleHipRatioX,
  (float)(0.7f) ankleHipRatioY
);

STREAMABLE(COMShiftParameters,,
  (float)(0.f) accXAlpha,
  (float)(0.f) stepAccAlpha,
  (int)(200) accInterpolTime
);

//struct FreeLegPhaseParams : public WalkingEngineParams { };
