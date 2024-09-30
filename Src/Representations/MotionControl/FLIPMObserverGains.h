#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

STREAMABLE(FLIPMObserverGains,
	ENUM(CoMProvider,
	  targetCoM,
	  MRECoM,
	  IMUCoM
	);

	ENUM(ACCProvider,
	  targetAcc,
	  ZMPAcc,
	  ZMPCoM1Acc,
    ZMPTargetAcc,
	  IMUAcc
	);
	,
	(bool)(false) activateSensorX,
	(float)(0.f) sensorXFactor,
	(float[3]) sensorControlRatioObserverX,
	(bool)(false) activateSensorY,
	(float)(0.f) sensorYFactor,
	(float[3]) sensorControlRatioObserverY,
	(float)(0.f) maxDiffCoMClip,
	(float)(0.f) maxDiffACCClip,
	(CoMProvider) CoM1Provider,
	(ACCProvider) ACC1Provider,
	(CoMProvider) CoM2Provider,
	(unsigned int)(0) CoM1Delay,
	(float)(0.f) CoM1OffsetX,
	(float)(0.f) CoM1OffsetY,

	(unsigned int)(0) CoM2Delay,
	(float)(0.f) CoM2OffsetX,
	(float)(0.f) CoM2OffsetY,
	(float)(0.f) CoM2DiffXOffset,
	(float)(0.f) CoM2DiffYOffset,

	(float)(0.f) accFilter,
	(bool)(false) useRCS
);
