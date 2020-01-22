#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

#define PREVIEW_LENGTH 100 /**< Length of the preview phase */

//STREAMABLE(FLIPMValues,
//{ ,
//  (float) m,
//  (float) M,
//  (float) g,
//  (float) z_h,
//  (float) dt,
//  (float) D,
//  (float) E,
//  (float) Qe,
//  (float) Qx,
//  (float) R,
//  (Matrix6d) Ql,
//  (Matrix3d) RO,
//});

struct FLIPMValues : public Streamable
{
  float m;
  float M;
  float g;
  float z_h;
  float dt;
  float D;
  float E;
  float Qe;
  float Qx;
  float R;
  Matrix6f Ql;
  Matrix3f RO;
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(m)
      STREAM(M)
      STREAM(g)
      STREAM(z_h)
      STREAM(dt)
      STREAM(D)
      STREAM(E)
      STREAM(Qe)
      STREAM(Qx)
      STREAM(R)
      STREAM(Ql)
      STREAM(RO)
    STREAM_REGISTER_FINISH;
  };
};

STREAMABLE(FLIPMControllerValues,
{ ,
  (Matrix6d) A,
  (Vector6d) b,
  (Matrix1x6d) c,
  (double) Gi,
  (Matrix1x6d) Gx,
  (Eigen::Matrix<double, PREVIEW_LENGTH, 1>) Gd,
});

STREAMABLE(FLIPMObserverValues,
{ ,
  (Matrix6x3d) L,
});

STREAMABLE(FLIPMParameter,
{ ,
  (FLIPMValues) paramsX,
  (FLIPMValues) paramsY,
});

STREAMABLE(FLIPMControllerParameter,
{ ,
  (bool)(false) useRCS,
  (bool)(false) useRefZMPInterpolation, // for reset of preview, bc refZMP can jump
  (FLIPMControllerValues) controllerParamsX,
  (FLIPMControllerValues) controllerParamsY,
});

STREAMABLE(FLIPMObserverParameter,
{ ,
  (bool)(false) useIMUModel,
  (FLIPMObserverValues) observerParamsX,
  (FLIPMObserverValues) observerParamsY,
});
