#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Enum.h"

class FLIPMObserverParams : public Streamable
{
public:
	ENUM(CoMProvider,
	{ ,
	  targetCoM,
	  MRE_CoM,
	  IMU_CoM,
	});

	ENUM(ACCProvider,
	{ ,
	  targetAcc,
	  ZMP_Acc,
	  ZMP_CoM1_Acc,
	  IMU_Acc,
	});

	bool            activateSensorX;
  float           sensorXFactor;
	float           sensorControlRatioObserverX[3];
	bool            activateSensorY;
  float           sensorYFactor;
	float           sensorControlRatioObserverY[3];
	float			maxDiffCoMClip;
	float			maxDiffACCClip;
	CoMProvider     CoM1Provider;
	ACCProvider     ACC1Provider;
	CoMProvider     CoM2Provider;
	unsigned int    CoM1Delay;
	float           CoM1OffsetX;
	float           CoM1OffsetY;

	unsigned int    CoM2Delay;
	float           CoM2OffsetX;
	float           CoM2OffsetY;
	float           CoM2DiffXOffset;
	float           CoM2DiffYOffset;

	float           accFilter;
	bool            useRCS;

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(activateSensorX);
    STREAM(sensorXFactor);
    STREAM(sensorControlRatioObserverX);
    STREAM(activateSensorY);
    STREAM(sensorYFactor);
    STREAM(sensorControlRatioObserverY);
    STREAM(maxDiffCoMClip);
    STREAM(maxDiffACCClip);
    STREAM(CoM1Provider);
    STREAM(ACC1Provider);
    STREAM(CoM2Provider);
    STREAM(CoM1Delay);
    STREAM(CoM1OffsetX);
    STREAM(CoM1OffsetY);
    STREAM(CoM2Delay);
    STREAM(CoM2OffsetX);
    STREAM(CoM2OffsetY);
    STREAM(CoM2DiffXOffset);
    STREAM(CoM2DiffYOffset);
    STREAM(accFilter);
    STREAM(useRCS);
    STREAM_REGISTER_FINISH;
  }

};