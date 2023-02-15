#include "KeyStates.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

KeyStates::KeyStates()
{
  pressed.fill(false);
}

void KeyStates::draw() const
{
  //DECLARE_PLOT("representation:KeyStates:headFront");
  //DECLARE_PLOT("representation:KeyStates:headMiddle");
  //DECLARE_PLOT("representation:KeyStates:headRear");

  //DECLARE_PLOT("representation:KeyStates:lHandBack");
  //DECLARE_PLOT("representation:KeyStates:lHandLeft");
  //DECLARE_PLOT("representation:KeyStates:lHandRight");

  //DECLARE_PLOT("representation:KeyStates:rHandBack");
  //DECLARE_PLOT("representation:KeyStates:rHandLeft");
  //DECLARE_PLOT("representation:KeyStates:rHandRight");

  //DECLARE_PLOT("representation:KeyStates:leftFootLeft");
  //DECLARE_PLOT("representation:KeyStates:leftFootRight");

  //DECLARE_PLOT("representation:KeyStates:rightFootLeft");
  //DECLARE_PLOT("representation:KeyStates:rightFootRight");

  //DECLARE_PLOT("representation:KeyStates:chest");

  PLOT("representation:KeyStates:headFront", pressed[headFront]);
  PLOT("representation:KeyStates:headMiddle", pressed[headMiddle]);
  PLOT("representation:KeyStates:headRear", pressed[headRear]);

  PLOT("representation:KeyStates:lHandBack", pressed[lHandBack]);
  PLOT("representation:KeyStates:lHandLeft", pressed[lHandLeft]);
  PLOT("representation:KeyStates:lHandRight", pressed[lHandRight]);

  PLOT("representation:KeyStates:rHandBack", pressed[lHandRight]);
  PLOT("representation:KeyStates:rHandLeft", pressed[rHandLeft]);
  PLOT("representation:KeyStates:rHandRight", pressed[rHandRight]);

  PLOT("representation:KeyStates:leftFootLeft", pressed[leftFootLeft]);
  PLOT("representation:KeyStates:leftFootRight", pressed[leftFootRight]);

  PLOT("representation:KeyStates:rightFootLeft", pressed[rightFootLeft]);
  PLOT("representation:KeyStates:rightFootRight", pressed[rightFootRight]);

  PLOT("representation:KeyStates:chest", pressed[chest]);
}
