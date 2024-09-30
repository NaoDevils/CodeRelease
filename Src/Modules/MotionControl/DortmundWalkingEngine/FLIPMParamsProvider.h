#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/WalkCalibration.h"
#include "Tools/Streams/RobotParameters.h"
#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <future>

constexpr unsigned MODEL_STATE_DIM = 6;
constexpr unsigned MODEL_CONTROL_DIM = 3;

MODULE(FLIPMParamsProvider,
  REQUIRES(FrameInfo),
  REQUIRES(WalkCalibration),
  PROVIDES_WITHOUT_MODIFY(FLIPMParameter),
  PROVIDES(FLIPMControllerParameter),
  PROVIDES(FLIPMObserverParameter),

  LOADS_PARAMETERS(,
    (bool)(false) useRCS,
    (bool)(false) autoRecalculate,
    (bool)(true) duplicateXParams,
    (FLIPMValues) paramsX,
    (FLIPMValues) paramsY
  )
);

class FLIPMParamsProvider : public FLIPMParamsProviderBase
{

public:
  FLIPMParamsProvider();

  class LQRParams
  {
  public:
    LQRParams() { clear(); }

    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
    Eigen::MatrixXd c;
    double Gi;
    Eigen::MatrixXd Gx;
    Eigen::MatrixXd Gd;
    Eigen::MatrixXd L;
    bool valid;
    bool controllable;
    bool observable;
    double calculationTime;

    void clear()
    {
      A = Eigen::MatrixXd::Zero(MODEL_STATE_DIM, MODEL_STATE_DIM);
      b = Eigen::MatrixXd::Zero(MODEL_STATE_DIM, 1);
      c = Eigen::MatrixXd::Zero(1, MODEL_STATE_DIM);
      Gi = 0.0;
      Gx = Eigen::MatrixXd::Zero(1, MODEL_STATE_DIM);
      Gd = Eigen::MatrixXd::Zero(PREVIEW_LENGTH, 1);
      L = Eigen::MatrixXd::Zero(MODEL_STATE_DIM, MODEL_CONTROL_DIM);
      valid = true;
      controllable = true;
      observable = true;
      calculationTime = 0.0;
    }
  };

protected:
  ENUM(Dimension,
    X,
    Y
  );

  FLIPMValues lastParamsX, lastParamsY;

  LQRParams xLQRParams;
  LQRParams yLQRParams;

  std::future<LQRParams> xLQRParamsFuture;
  std::future<LQRParams> yLQRParamsFuture;

  bool initializedFLIPMParams = false;
  FLIPMParameter lastFLIPMParams;

  void update(FLIPMParameter& flipmParameter)
  {
    flipmParameter.paramsX = paramsX;
    flipmParameter.paramsY = paramsY;
  }
  void update(FLIPMControllerParameter& flipmControllerParameter);
  void update(FLIPMObserverParameter& flipmObserverParameter);
  void recalculate(Dimension dim);

  LQRParams execute(Dimension dim);
  LQRParams executeX() { return execute(X); }
  LQRParams executeY() { return execute(Y); }

  /// Returns absolute elementwise @p tolerance.
  /// Special values (infinities, NaN, etc.) do not compare as equal elements.
  double abstol(const Eigen::Ref<const Eigen::MatrixXd>& X, const Eigen::Ref<const Eigen::MatrixXd>& Y)
  {
    ASSERT((X.rows() == Y.rows() && X.cols() == Y.cols()) && "X & Y have not the same dimensions");
    return (X - Y).lpNorm<Eigen::Infinity>();
  }

  bool checkObservability(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& c) const;
  bool checkControllability(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b) const;

  void Givens_rotation(double a, double b, Eigen::Ref<Eigen::Matrix2d> R, double eps);
  void swap_block_11(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p);
  void swap_block_21(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p);
  void swap_block_12(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p);
  void swap_block_22(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p);
  void swap_block(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p, int q, int q_block_size, double eps);
  void reorder_eigen(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, double eps);

  Eigen::MatrixXd solveDARE_analytic(
      const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R);

  Eigen::MatrixXd dlqr(
      const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R);
};
