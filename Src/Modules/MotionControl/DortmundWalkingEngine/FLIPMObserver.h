/**
 * @file FLIPMObserver.h
 * @author <a href="mailto:arne.moos@tu-dortmund.de> Arne Moos</a>
 */

#pragma once
/* tells the RingBuffer to check the boundaries */
#define LIMIT_CHECK

#include <list>
#include <stdio.h>
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/Streams/RobotParameters.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/ObservedFLIPMError.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/ActualCoM.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/MotionControl/FootSteps.h"

//#include "Tools/Math/Pose3D.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Configuration/RobotDimensions.h"
//#include "Tools/MathDortmund/Pose3D_D.h"

//#include "Tools/Streams/InStreams.h"

#define COMMA ,
const int static_N = 50; /**< Length of the preview phase */
#define SEPERATEDXY 1

MODULE(FLIPMObserver,
{ ,
  REQUIRES(WalkingEngineParams),
  REQUIRES(ActualCoM),
  REQUIRES(ActualCoMFLIPM),
  REQUIRES(PatternGenRequest),
  REQUIRES(InertialSensorData),
  REQUIRES(ZMPModel),
  REQUIRES(FootSteps),
  /////////////////////////////
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  /////////////////////////////
  USES(WalkingInfo),
  USES(TargetCoM),
  /////////////////////////////
  PROVIDES(ObservedFLIPMError),
  //LOADS_PARAMETERS(
  //{ ,
  //  // TODO: initialized has to be changed when changing parameters!!
  //  (bool)  activateSensorX,
  //  (float) sensorControlRatioObserverX[3],
  //  (bool)  activateSensorY,
  //  (float) sensorControlRatioObserverY[3],
  //  (int)   FLIPM_XOffset,
  //  (float) FLIPM_IMUFilter,
  //  (int)   FLIPM_CoM1Delay,
  //  (float) FLIPM_CoM1Multiplikator[2],
  //  (float) FLIPM_CoM1OffsetX,
  //  (float) FLIPM_CoM1OffsetY,
  //  (float) FLIPM_CoM1DiffOffsetX,
  //  (float) FLIPM_CoM1DiffOffsetY,
  //  (int)   FLIPM_CoM2Delay,
  //  (float) FLIPM_CoM2Multiplikator[2],
  //  (float) FLIPM_CoM2OffsetX,
  //  (float) FLIPM_CoM2OffsetY,
  //  (float) FLIPM_CoM2DiffOffsetX,
  //  (float) FLIPM_CoM2DiffOffsetY,
  //  (int)   FLIPM_ACC1Provider,
  //  (int)   FLIPM_CoM1Provider,
  //  (int)   FLIPM_CoM2Provider,
  //  (int)   FLIPM_CoM2DiffProvider,
  //}),
});

class FLIPMObserverParams : public Streamable
{
public:
	FLIPMObserverParams() {
    FILE *stream;
    std::string name = "flipmObserverParams";
    std::string fullPath;
    std::list<std::string> names = File::getFullNames(name + ".cfg");
    bool found = false;

    for (auto& path : names)
    {
      stream = fopen(path.c_str(), "r");
      if (stream)
      {
        found = true;
        fullPath = path;
        break;
      }
    }
    if (!found)
    {
      fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg");
      ASSERT(false);
    }
    fclose(stream);
    InMapFile file(name + std::string(".cfg"));
    if (file.exists())
      file >> *this;
    else
      OUTPUT_TEXT("Warning, could not create" + fullPath);
	}

	bool activateSensorX;
	float sensorControlRatioObserverX[3];
	bool activateSensorY;
	float sensorControlRatioObserverY[3];
	int FLIPM_XOffset;
	float FLIPM_IMUFilter;
  int FLIPM_CoM1Delay;
  float FLIPM_CoM1Multiplikator[2];
	float FLIPM_CoM1OffsetX;
	float FLIPM_CoM1OffsetY;
  float FLIPM_CoM1DiffOffsetX;
  float FLIPM_CoM1DiffOffsetY;
  int FLIPM_CoM2Delay;
  float FLIPM_CoM2Multiplikator[2];
	float FLIPM_CoM2OffsetX;
	float FLIPM_CoM2OffsetY;
  float FLIPM_CoM2DiffOffsetX;
  float FLIPM_CoM2DiffOffsetY;
	int FLIPM_ACC1Provider;
	int FLIPM_CoM1Provider;
	int FLIPM_CoM2Provider;
	int FLIPM_CoM2DiffProvider;

	void serialize(In* in, Out* out)
	{
		STREAM_REGISTER_BEGIN;
		STREAM(activateSensorX);
		STREAM(sensorControlRatioObserverX);
		STREAM(activateSensorY);
		STREAM(sensorControlRatioObserverY);
		STREAM(FLIPM_XOffset);
		STREAM(FLIPM_IMUFilter);
    STREAM(FLIPM_CoM1Delay);
    STREAM(FLIPM_CoM1Multiplikator);
		STREAM(FLIPM_CoM1OffsetX);
		STREAM(FLIPM_CoM1OffsetY);
    STREAM(FLIPM_CoM1DiffOffsetX);
    STREAM(FLIPM_CoM1DiffOffsetY);
    STREAM(FLIPM_CoM2Delay);
    STREAM(FLIPM_CoM2Multiplikator);
		STREAM(FLIPM_CoM2OffsetX);
		STREAM(FLIPM_CoM2OffsetY);
    STREAM(FLIPM_CoM2DiffOffsetX);
    STREAM(FLIPM_CoM2DiffOffsetY);
		STREAM(FLIPM_ACC1Provider);
		STREAM(FLIPM_CoM1Provider);
		STREAM(FLIPM_CoM2Provider);
		STREAM(FLIPM_CoM2DiffProvider);
		STREAM_REGISTER_FINISH;
	}
};

class FLIPMObserver : public FLIPMObserverBase
{
  ROBOT_PARAMETER_CLASS(FLIPMObsvX, FLIPMObserver)
	  PARAM(int, N)
	  PARAM(float, z_h)
	  PARAM(float, g)
	  PARAM(Matrix6x3f, L)
  END_ROBOT_PARAMETER_CLASS(FLIPMObsvX)

  ROBOT_PARAMETER_CLASS(FLIPMObsvY, FLIPMObserver)
    PARAM(int, N)
    PARAM(float, z_h)
    PARAM(float, g)
    PARAM(Matrix6x3f, L)
  END_ROBOT_PARAMETER_CLASS(FLIPMObsvY)



public:
	FLIPMObserver():
		isStable(true),
		localSensorScale(0),
		filteredAccX(0.0),
		filteredAccY(0.0)
	{};

	void update(ObservedFLIPMError &observedFLIPMError);

protected:
	FLIPMObserverParams flipmObserverParams;
private:

  RingBuffer<Vector2f, static_N> accDelayBuffer,	/**< Buffer to deal with the sensor delay. */
		coM1DelayBuffer,									/**< Buffer to deal with the sensor delay. */
		coM2DelayBuffer,									/**< Buffer to deal with the sensor delay. */	
    realCoM1DelayBuffer,
    realCoM2DelayBuffer;

	bool isStable;
	float localSensorScale;
	Point lastRealZMP;

	double filteredAccX;
	double filteredAccY;
};

