#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

#define PREVIEW_LENGTH 100 /**< Length of the preview phase */

STREAMABLE(FLIPMValues,
  bool equal(const FLIPMValues& flipmValues)
  {
    if (m == flipmValues.m &&
      M == flipmValues.M &&
      z_h == flipmValues.z_h &&
      D == flipmValues.D &&
      E == flipmValues.E &&
      Qe == flipmValues.Qe &&
      Qx == flipmValues.Qx &&
      R == flipmValues.R &&
      Ql.isApprox(flipmValues.Ql) &&
      RO.isApprox(flipmValues.RO))
      return true;
    return false;
  }
  ,
  (float)(0.f) m,
  (float)(0.f) M,
  (float)(0.f) z_h,
  (float)(0.f) D,
  (float)(0.f) E,
  (float)(0.f) Qe,
  (float)(0.f) Qx,
  (float)(0.f) R,
  (Matrix6f) Ql,
  (Matrix3f) RO
);

STREAMABLE(FLIPMControllerValues,,
  (Matrix6d) A,
  (Vector6d) b,
  (Matrix1x6d) c,
  (double) Gi,
  (Matrix1x6d) Gx,
  (Eigen::Matrix<double, PREVIEW_LENGTH, 1>) Gd
);

STREAMABLE(FLIPMObserverValues,,
  (Matrix6x3d) L
);

STREAMABLE(FLIPMParameter,,
  (FLIPMValues) paramsX,
  (FLIPMValues) paramsY
);

STREAMABLE(FLIPMControllerParameter,,
  (bool)(false) useRCS,
  (bool)(false) useRefZMPInterpolation, // for reset of preview, bc refZMP can jump
  (int)(15) framesToInterpolate,
  (FLIPMControllerValues) controllerParamsX,
  (FLIPMControllerValues) controllerParamsY
);

STREAMABLE(FLIPMObserverParameter,,
  (FLIPMObserverValues) observerParamsX,
  (FLIPMObserverValues) observerParamsY
);
