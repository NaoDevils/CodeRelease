/**
* @file Representations/MotionControl/ArmContact.h
* This file declares a class that represents if an ArmContact exists.
* @author <a href="mailto:dino.menges@tu-dortmund.de> Dino Menges</a>
*/

#ifndef __ArmContact_H__
#define __ArmContact_H__

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"
#else
#include "bhumanstub.h"
#endif

/**
* @class ArmContact
* A class that represents if an ArmContact exists.
*/
class ArmContact : public Streamable
{
protected:
  virtual void serialize(In* in, Out* out)
  {  
    STREAM_REGISTER_BEGIN;
    STREAM(timeStampLeft);
    STREAM(timeStampRight);
    STREAM(armContactStateLeft);
    STREAM(armContactStateRight);
    STREAM_REGISTER_FINISH;
  }

public:
  ENUM(ArmContactState,
  { ,
    None,
    Front,
    Back,
  });

  /** 
  * Default constructor.
  */
  ArmContact() : timeStampRight(0),timeStampLeft(0),armContactStateLeft(None),armContactStateRight(None) {}

  unsigned timeStampRight, timeStampLeft;
  ArmContactState armContactStateLeft, armContactStateRight;

};
#endif // __ArmContact_H__
