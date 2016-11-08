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
struct KinematicRequest : public Streamable, public Watch
{
private:
  virtual void serialize( In* in, Out* out)
  { 
    STREAM_REGISTER_BEGIN;
    STREAM(kinematicType);
    STREAM(offsets);
    STREAM(body);
    STREAM(leftFoot);
    STREAM(rightFoot);
    STREAM_REGISTER_FINISH;
  }

public:

	void watch()
	{
		WATCH_ARRAY(leftFoot);
		WATCH_ARRAY(rightFoot);
		WATCH_ARRAY(offsets.angles);
	}

  ENUM(KinematicType,
  {,
    feet,              // Position of the feet relative to the body.
    bodyAndLeftFoot,    // Position of the body and left foot relative to the right foot.
    bodyAndRightFoot,
  }// Position of the body and right foot relative to the left foot.
  );
  


  enum armEnum{
	  left0,
	  left1,
	  left2,
	  left3,
	  right0,
	  right1,
	  right2,
	  right3,
	  numOfArmAngles
  };
	  
  KinematicRequest(){  
    int i;
    for(i = 0; i < Joints::numOfJoints; ++i) offsets.angles[i] = 0;
    for (i = 0; i < 6; ++i) {
      body[i] = 0;
      leftFoot[i] = 0;
      rightFoot[i] = 0;
    }
  }

  /** kinematic type */
  KinematicType kinematicType;

  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float body[6]; 
  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float leftFoot[6];
  /** There are the desired foot/body positions x,y,z, rot-x, rot-y, rot-z. */
  float rightFoot[6];

  JointAngles offsets;
};

