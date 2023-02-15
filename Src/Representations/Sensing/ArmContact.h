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
STREAMABLE(ArmContact,
  ENUM(ArmContactState,
    None,
    Front,
    Back
  );
  ,
  (unsigned)(0) timeStampLeft,
  (unsigned)(0) timeStampRight,
  (ArmContactState)(None) armContactStateLeft,
  (ArmContactState)(None) armContactStateRight
);
#endif // __ArmContact_H__
