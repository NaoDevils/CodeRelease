#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

#define PREVIEW_LENGTH 50 /**< Length of the preview phase */

STREAMABLE(ParamsFLIPM,
{ ,
  (float) m,
  (float) M,
  (float) g,
  (float) z_h,
  (float) dt,
  (float) D,
  (float) E,
  (float) Qe,
  (float) Qx,
  (float) R,
  (int) N,
  (Matrix6f) A,
  (Vector6f) b,
  (Matrix1x6f) c,
  (float) Gi,
  (Matrix1x6f) Gx,
  (Eigen::Matrix<float, PREVIEW_LENGTH, 1>) Gd,
  (Matrix6x3f) L,
});

STREAMABLE(FLIPMControllerParams,
{ ,
  (bool)(false) useRCS,
  (ParamsFLIPM) paramsX,
  (ParamsFLIPM) paramsY,
});
