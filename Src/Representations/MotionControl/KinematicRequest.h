#pragma once

#include "Representations/Infrastructure/JointAngles.h"

#ifdef WALKING_SIMULATOR
#include "bhumanstub.h"
#include "Watch.h"
#else
#include "Tools/Debugging/Watch.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#endif

//#define USEARMS

/**
* @class KinematicRequest
* A class that represents a kinematic request.
*/
STREAMABLE(KinematicRequest,
	void watch()
	{
		WATCH_ARRAY(leftFoot);
		WATCH_ARRAY(rightFoot);
		WATCH_ARRAY(offsets.angles);
	}

  ENUM(KinematicType,
    feet,              // Position of the feet relative to the body.
    bodyAndLeftFoot,    // Position of the body and left foot relative to the right foot.
    bodyAndRightFoot   // Position of the body and right foot relative to the left foot.
  );
  
  KinematicRequest(){  
    int i;
    for(i = 0; i < Joints::numOfJoints; ++i) offsets.angles[i] = 0;
    for (i = 0; i < 6; ++i) {
      leftFoot[i] = 0;
      rightFoot[i] = 0;
    }
  }
  ,
  /** kinematic type */
  (KinematicType) kinematicType,

  (JointAngles) offsets,

  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  (float[6]) leftFoot,
  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  (float[6]) rightFoot
);
