/**
* @file ActualAcc
* @author <a href="mailto:arne.moos@tu-dortmund.de>Arne Moos</a>
*/
#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Modules/MotionControl/DortmundWalkingEngine/Point.h"


STREAMABLE(FLIPMObservedState,,
  (Vector3f)(Vector3f::Zero()) actualAcc,
  (TranslationPoint) actualCoM_IMU,
  (TranslationPoint) actualCoM_MRE
);
