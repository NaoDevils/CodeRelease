#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/KeySymbols.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/RingBufferWithSum.h"
#include "Platform/SystemCall.h"

MODULE(KeySymbolsProvider,
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  PROVIDES(KeySymbols),
  LOADS_PARAMETERS(,
    (float) minBufferAvgForObstacle
  )
);

/**
*
* @author Ingmar Schwarz
*/
class KeySymbolsProvider : public KeySymbolsProviderBase
{
public:
  /*
  * Constructor.
  * @param frameInfo A reference to the FrameInfo
  * @param switchState A reference to the SwitchState.
  */
  KeySymbolsProvider() : timeWhenObstacleHit(0)
  {
    for (int i = 0; i < KeyStates::numOfKeys; i++)
    {
      last_pressed_time[i] = 0;
    }
  }

  unsigned last_pressed_time[KeyStates::numOfKeys];
  unsigned timeWhenObstacleHit;

  void update(KeySymbols& keySymbols);

private:
  RingBufferWithSum<float, 10> pressBuffer;
  RingBufferWithSum<float, 600> buttonCheckFilter[4];
};
