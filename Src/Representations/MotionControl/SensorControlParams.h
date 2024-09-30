/**
* @class SensorControlParams
* Combines the parameters of all sensor controls in the walking engine.
* @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
*/
#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Angle.h"
#include "Representations/Sensing/JoinedIMUData.h"
#include "Representations/MotionControl/FLIPMObserverGains.h"

STREAMABLE(PID,,
  (float)(0.f) P,
  (float)(0.f) I,
  (float)(0.f) D
);

STREAMABLE(Multiplicators,,
  (float)(0.f) zero,
  (float)(0.f) normal
);

STREAMABLE(AdvancedPID,,
  (Multiplicators) pidMultiplicatorsX, // zero and normal
  (PID) x,
  (Multiplicators) pidMultiplicatorsY, // zero and normal
  (PID) y
);

STREAMABLE(PD,,
  (float)(0.f) P,
  (float)(0.f) D
);

STREAMABLE(MinMaxFloat,,
  (float)(0.f) min,
  (float)(0.f) max
);

STREAMABLE(MinMaxAngle,,
  (Angle)(0_deg) min,
  (Angle)(0_deg) max
);

//////////////////////////////////////////////////////////////////////////////////////
STREAMABLE(SensorControlActivation,,
  (bool)(true) activateParalellFeetController,
  (bool)(true) activateJointErrorPID,
  (bool)(true) activateAnkleHipPID,
  (bool)(true) activateCoMShifting,
  (bool)(true) activateStepDurationAdjustment,
  (bool)(true) activateSpeedReduction,
  (bool)(true) activatePreviewController,
  (bool)(false) activateSafetySteps,
  (bool)(false) activateFLIPMObserverGains,
  (bool)(false) activateCalibrationScaling
);
//////////////////////////////////////////////////////////////////////////////////////

STREAMABLE(StepDurationAdjustment,,
  (MinMaxFloat) stepDuration, // min / max step duration
  (float)(0.95f) exponentialFactor
);

STREAMABLE(ParalellFeetController,,
  (PD) controller, // pitch and roll control for x and y
  (float[5]) polygon,
  ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) anglesource
);

STREAMABLE(SpeedReduction,,
  (MinMaxAngle) angleY, // min / max speed reduction angle around y axis
  (MinMaxAngle) angleX,  // min / max speed reduction angle around x axis
  (float)(1.05f) speedReductionFactor,
  (float)(1.15f) speedReductionExponentialBasis,
  (float)(1.65f) emergencyStopSpeedReductionFactorThreshold,
  (float)(0.70f) emergencyStopGyroVariance,
  (Vector2f)({0.25f, 0.25f}) stepSpeedInfluenceFactor,
  (float)(2.5f) customStepFactorStable,
  ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) anglesource
);

STREAMABLE(AnkleHipPID,,
  (Vector2a)(Vector2a::Zero()) targetAngle, // x / y target angle
  (AdvancedPID) ankleParams,
  (AdvancedPID) hipParams,
  (Vector2f)(Vector2f::Zero()) ankleHipRatio, // x / y ankleHipRatio
  (bool)(false) balanceSupportLegOnly,
  ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) anglesource
);

STREAMABLE(JointErrorPID,,
  (PID) pid // JointError pid influence
);

STREAMABLE(CoMShifting,,
  (float)(-0.75) accXAlpha,
  (float)(0.f) stepAccAlpha,
  (int)(200) accInterpolTime,
  ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) anglesource
);

STREAMABLE(PreviewController,,
  (bool)(true) freezeDSRefZMPUntilFSR,
  (bool)(false) slowDSRefZMPUntilFSR,
  (int)(50) maxFramesForDSExtension
);

STREAMABLE(SafetySteps,,
  (int)(500) angleSumForSafetyStepFront,
  (int)(150) angleSumForSafetyStepBack,
  (float)(2.f) angleSumDecay,
  (float)(10.f) comErrorFrontForSafetySteps,
  (float)(-5.f) comErrorBackForSafetySteps,
  (Vector2f)(Vector2f(30.f,20.f)) maxSafetyStepCorrection,
  ((JoinedIMUData) InertialDataSource)(JoinedIMUData::imuModel) anglesource
);

//////////////////////////////////////////////////////////////////////////////////////
STREAMABLE(SensorControlParams,,
  (SensorControlActivation) sensorControlActivation,
  (ParalellFeetController) paralellFeetController,
  (JointErrorPID) jointErrorPID,
  (AnkleHipPID) ankleHipPID,
  (CoMShifting) coMShifting,
  (StepDurationAdjustment) stepDurationAdjustment,
  (SpeedReduction) speedReduction,
  (PreviewController) previewController,
  (FLIPMObserverGains) flipmObserverGains,
  (SafetySteps) safetySteps
);
