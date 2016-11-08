/**
* @file Representations/MotionControl/KinematicOutput.h
* This file declares a class that represents the output of modules generating motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#ifndef __KinematicOutput_H__
#define __KinematicOutput_H__

#include "Representations/Infrastructure/JointRequest.h"

#ifndef WALKING_SIMULATOR
#include "Tools/Math/Pose2f.h"
#else
#include "math/Pose2D.h"
#include "bhumanstub.h"
#endif

/**
* @class KinematicOutput
* A class that represents the output of the walking engine.
*/
class KinematicOutput : public JointRequest
{
protected:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN;
    STREAM_BASE(JointRequest);
    STREAM_REGISTER_FINISH;
  }

public:
  /** 
  * Default constructor.
  */
  KinematicOutput() /*positionInWalkCycle(0)*/ {}
};
#endif // __KinematicOutput_H__
