#pragma once

#include "Representations/BehaviorControl/HeadPOIList.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(BallchaserHeadPOIList, HeadPOIList,
  ,
  (bool)(false) followKick,
  (bool)(false) playerShootsNow,
  (bool)(false) opponentShootsNow,
  (bool)(false) playerShootsSoon,
  (bool)(false) opponentShootsSoon
);
