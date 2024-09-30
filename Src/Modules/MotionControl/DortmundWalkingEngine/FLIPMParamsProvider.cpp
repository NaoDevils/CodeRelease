#include "FLIPMParamsProvider.h"
#include "Platform/SystemCall.h"
#include <iostream>

FLIPMParamsProvider::FLIPMParamsProvider()
{
  xLQRParams.clear();
  yLQRParams.clear();

  FLIPMControllerParameter loadedFLIPMControllerParameter;
  InMapFile fileC("flipmControllerParameter.cfg");
  if (fileC.exists())
    fileC >> loadedFLIPMControllerParameter;
  else
  {
    ASSERT(false && "Could not load flipmControllerParameter.cfg");
  }

  FLIPMObserverParameter loadedFLIPMObserverParameter;
  InMapFile fileO("flipmObserverParameter.cfg");
  if (fileO.exists())
    fileO >> loadedFLIPMObserverParameter;
  else
  {
    ASSERT(false && "Could not load flipmObserverParameter.cfg");
  }

  xLQRParams.A = loadedFLIPMControllerParameter.controllerParamsX.A;
  xLQRParams.b = loadedFLIPMControllerParameter.controllerParamsX.b;
  xLQRParams.c = loadedFLIPMControllerParameter.controllerParamsX.c;
  xLQRParams.Gi = loadedFLIPMControllerParameter.controllerParamsX.Gi;
  xLQRParams.Gd = loadedFLIPMControllerParameter.controllerParamsX.Gd;
  xLQRParams.Gx = loadedFLIPMControllerParameter.controllerParamsX.Gx;
  xLQRParams.L = loadedFLIPMObserverParameter.observerParamsX.L;

  yLQRParams.A = loadedFLIPMControllerParameter.controllerParamsY.A;
  yLQRParams.b = loadedFLIPMControllerParameter.controllerParamsY.b;
  yLQRParams.c = loadedFLIPMControllerParameter.controllerParamsY.c;
  yLQRParams.Gi = loadedFLIPMControllerParameter.controllerParamsY.Gi;
  yLQRParams.Gd = loadedFLIPMControllerParameter.controllerParamsY.Gd;
  yLQRParams.Gx = loadedFLIPMControllerParameter.controllerParamsY.Gx;
  yLQRParams.L = loadedFLIPMObserverParameter.observerParamsY.L;

  lastParamsX = paramsX;
  lastParamsY = paramsY;
}

void FLIPMParamsProvider::recalculate(Dimension dim)
{
  if (dim == X)
  {
    if (!xLQRParamsFuture.valid())
      xLQRParamsFuture = std::async(std::launch::async,
          [this]
          {
            return executeX();
          });
  }
  else if (dim == Y)
  {
    if (!yLQRParamsFuture.valid())
      yLQRParamsFuture = std::async(std::launch::async,
          [this]
          {
            return executeY();
          });
  }
}

void FLIPMParamsProvider::update(FLIPMControllerParameter& flipmControllerParameter)
{
  if (autoRecalculate)
  {
    if (!paramsX.equal(lastParamsX))
    {
      recalculate(X);
    }
    if (!paramsY.equal(lastParamsY))
    {
      recalculate(Y);
    }
  }

  DEBUG_RESPONSE_ONCE("module:FLIPMParamsProvider:recalculate:X")
  {
    recalculate(X);
  }

  DEBUG_RESPONSE_ONCE("module:FLIPMParamsProvider:recalculate:Y")
  {
    recalculate(Y);
  }

  if (xLQRParamsFuture.valid() && xLQRParamsFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    xLQRParams = xLQRParamsFuture.get();
    if (xLQRParams.valid)
    {
      //SystemCall::playSound("allright.wav");
    }
    else
      SystemCall::playSound("doh.wav");

    if (!xLQRParams.controllable)
      OUTPUT_TEXT("Controllability is not given!!! Try to change parameters...");
    else if (!xLQRParams.observable)
      OUTPUT_TEXT("Observability is not given!!! Try to change parameters...");
    //else
    //  OUTPUT_TEXT("X: solveDARE took " << xLQRParams.calculationTime << " s");
  }

  if (yLQRParamsFuture.valid() && yLQRParamsFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    yLQRParams = yLQRParamsFuture.get();
    if (yLQRParams.valid)
    {
      //SystemCall::playSound("allright.wav");
    }
    else
      SystemCall::playSound("doh.wav");
    if (!yLQRParams.controllable)
      OUTPUT_TEXT("Controllability is not given!!! Try to change parameters...");
    else if (!yLQRParams.observable)
      OUTPUT_TEXT("Observability is not given!!! Try to change parameters...");
    //else
    //  OUTPUT_TEXT("Y: solveDARE took " << yLQRParams.calculationTime << " s");
  }

  if (xLQRParams.valid)
  {
    flipmControllerParameter.controllerParamsX.A = xLQRParams.A;
    flipmControllerParameter.controllerParamsX.b = xLQRParams.b;
    flipmControllerParameter.controllerParamsX.c = xLQRParams.c;
    flipmControllerParameter.controllerParamsX.Gi = xLQRParams.Gi;
    flipmControllerParameter.controllerParamsX.Gd = xLQRParams.Gd;
    flipmControllerParameter.controllerParamsX.Gx = xLQRParams.Gx;
  }

  if (!duplicateXParams && yLQRParams.valid)
  {
    flipmControllerParameter.controllerParamsY.A = yLQRParams.A;
    flipmControllerParameter.controllerParamsY.b = yLQRParams.b;
    flipmControllerParameter.controllerParamsY.c = yLQRParams.c;
    flipmControllerParameter.controllerParamsY.Gi = yLQRParams.Gi;
    flipmControllerParameter.controllerParamsY.Gd = yLQRParams.Gd;
    flipmControllerParameter.controllerParamsY.Gx = yLQRParams.Gx;
  }
  else if (xLQRParams.valid)
  {
    flipmControllerParameter.controllerParamsY.A = xLQRParams.A;
    flipmControllerParameter.controllerParamsY.b = xLQRParams.b;
    flipmControllerParameter.controllerParamsY.c = xLQRParams.c;
    flipmControllerParameter.controllerParamsY.Gi = xLQRParams.Gi;
    flipmControllerParameter.controllerParamsY.Gd = xLQRParams.Gd;
    flipmControllerParameter.controllerParamsY.Gx = xLQRParams.Gx;
  }

  lastParamsX = paramsX;
  lastParamsY = paramsY;
}

void FLIPMParamsProvider::update(FLIPMObserverParameter& flipmObserverParameter)
{
  if (xLQRParams.valid)
  {
    flipmObserverParameter.observerParamsX.L = xLQRParams.L;
  }

  if (!duplicateXParams && yLQRParams.valid)
    flipmObserverParameter.observerParamsY.L = yLQRParams.L;
  else if (xLQRParams.valid)
    flipmObserverParameter.observerParamsY.L = xLQRParams.L;
}

FLIPMParamsProvider::LQRParams FLIPMParamsProvider::execute(Dimension dim)
{
  const FLIPMValues& params = dim == Dimension::X ? paramsX : paramsY;

  LQRParams lqrParams;
  lqrParams.clear();

  double dt = theFrameInfo.cycleTime;
  double dt2 = (dt * dt) / 2;
  double dt3 = (dt * dt * dt) / 6;
  double z_h = params.z_h;
  double g = theWalkCalibration.gravity;

  double M = params.M;
  double m = params.m;
  double D = params.D;
  double E = params.E;

  double DM = D / M;
  double EM = E / M;
  double Dm = D / m;
  double Em = E / m;

  double Qe = params.Qe;
  double Qx = params.Qx;
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
  R << params.R;

  Eigen::MatrixXd Ql = params.Ql.cast<double>();
  Eigen::MatrixXd RO = params.RO.cast<double>();

  //////////////////////////////////////////////////////////////////////////
  lqrParams.A << 1, dt, dt2, 0, 0, 0, 0, 1, dt, 0, 0, 0, -DM, -EM, 0, DM, EM, 0, 0, 0, 0, 1, dt, dt2, Dm * dt, Em * dt, 0, -Dm * dt, 1 - Em * dt, dt, 0, 0, 0, 0, 0, 1;
  lqrParams.b << 0, 0, 0, dt3, dt2, dt;
  lqrParams.c << 1, 0, -z_h / g, 0, 0, 0;

  Eigen::MatrixXd Bt = Eigen::MatrixXd::Zero(7, 1);
  Bt << lqrParams.c * lqrParams.b, lqrParams.b;

  Eigen::MatrixXd It = Eigen::MatrixXd::Zero(7, 1);
  It(0, 0) = 1;

  Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(7, MODEL_STATE_DIM);
  Ft << lqrParams.c * lqrParams.A, lqrParams.A;

  Eigen::MatrixXd Qt = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd cT = lqrParams.c.transpose();
  Eigen::MatrixXd cTQx = cT * Qx;
  Eigen::MatrixXd cTQxC = cTQx * lqrParams.c;
  Qt(0, 0) = Qe;
  Qt.block(1, 1, 6, 6) = cTQxC;

  Eigen::MatrixXd At = Eigen::MatrixXd::Zero(7, 7);
  At.block(0, 0, 7, 1) = It;
  At.block(0, 1, 7, 6) = Ft;

  Eigen::MatrixXd Rt = Eigen::MatrixXd::Ones(1, 1);
  Rt << R;

  if (checkControllability(At, Bt))
  {
    Eigen::MatrixXd Pt = solveDARE_analytic(At, Bt, Qt, Rt);

    double RBttPtBt = (R + Bt.transpose() * Pt * Bt)(0);
    lqrParams.Gx = Eigen::Matrix<double, 1, 1>(1.0 / RBttPtBt) * (Bt.transpose() * Pt * Ft);
    lqrParams.Gi = (Eigen::Matrix<double, 1, 1>(1.0 / RBttPtBt) * (Bt.transpose() * Pt * It))(0);

    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(7, 7);
    Ac = At - Bt * Eigen::Matrix<double, 1, 1>(1.0 / RBttPtBt) * (Bt.transpose() * Pt * At);

    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(7, 1);
    X = -Ac.transpose() * Pt * It;

    std::vector<double> Gd_vec;
    Gd_vec.reserve(PREVIEW_LENGTH);
    Gd_vec.push_back(-lqrParams.Gi);
    for (int i = 1; i < PREVIEW_LENGTH; i++)
    {
      Gd_vec.push_back((Eigen::Matrix<double, 1, 1>(1.0 / RBttPtBt) * Bt.transpose() * X)(0));
      X = Ac.transpose() * X;
    }
    Eigen::MatrixXd Gd = Eigen::Map<Eigen::MatrixXd>(Gd_vec.data(), PREVIEW_LENGTH, 1);
    lqrParams.Gd = Gd;

    // OLD Observable States
    Eigen::MatrixXd Cm = Eigen::MatrixXd::Zero(3, 6);
    Cm(0, 0) = 1;
    Cm(1, 2) = 1;
    Cm(2, 3) = 1;

    if (checkObservability(lqrParams.A, Cm))
    {
      const unsigned long long startTime = SystemCall::getCurrentThreadTime();
      Eigen::MatrixXd resultingL = dlqr(lqrParams.A.transpose(), Cm.transpose(), Ql, RO);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> resultingL_rowmajor(resultingL);
      Eigen::Map<Eigen::MatrixXd> L(resultingL_rowmajor.data(), 6, 3);
      lqrParams.L = L;
      const unsigned long long stopTime = SystemCall::getCurrentThreadTime();

      const unsigned diff_time2 = unsigned(stopTime - startTime);
      lqrParams.calculationTime = (diff_time2 / 1000.0 / 1000.0);
    }
    else
    {
      lqrParams.valid = false;
      lqrParams.observable = false;
    }
  }
  else
  {
    lqrParams.valid = false;
    lqrParams.controllable = false;
  }

  return lqrParams;
}

bool FLIPMParamsProvider::checkObservability(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& c) const
{
  // calculate observability matrix
  ASSERT(c.rows() == 3);
  ASSERT(c.cols() == 6);
  Eigen::MatrixXd obsv(c.cols() * c.rows(), c.cols());

  obsv << c, c * A, c * (A * A), c * (A * A * A), c * (A * A * A * A), c * (A * A * A * A * A);

  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(obsv);
  lu_decomp.setThreshold(1e-10);
  int rank = static_cast<int>(lu_decomp.rank());

  return rank == A.rows();
  return true;
}

bool FLIPMParamsProvider::checkControllability(const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b) const
{
  // calculate controllability matrix
  ASSERT(b.cols() == 1);
  ASSERT(b.rows() == 7);
  Eigen::MatrixXd ctrb(b.rows(), b.rows());

  ctrb << b, A * b, (A * A) * b, (A * A * A) * b, (A * A * A * A) * b, (A * A * A * A * A) * b, (A * A * A * A * A * A) * b;

  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(ctrb);
  lu_decomp.setThreshold(1e-10);
  int rank = static_cast<int>(lu_decomp.rank());

  return rank == A.rows();
  return true;
}

Eigen::MatrixXd FLIPMParamsProvider::dlqr(
    const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R)
{
  Eigen::MatrixXd L = Eigen::MatrixXd::Zero(MODEL_STATE_DIM, MODEL_CONTROL_DIM);
  Eigen::MatrixXd bt = b.transpose();
  Eigen::MatrixXd S = solveDARE_analytic(A, b, Q, R);

  Eigen::MatrixXd btS = bt * S;
  Eigen::MatrixXd btSA = btS * A;
  Eigen::MatrixXd btSb = btS * b;
  Eigen::MatrixXd btSbR = R + btSb;
  Eigen::MatrixXd btSbR_inv = btSbR.inverse();

  Eigen::MatrixXd tempL = btSbR_inv * btSA;
  L = tempL.transpose();
  return L;
}

// "Givens rotation" computes an orthogonal 2x2 matrix R such that
// it eliminates the 2nd coordinate of the vector [a,b]', i.e.,
// R * [ a ] = [ a_hat ]
//     [ b ]   [   0   ]
// The implementation is based on
// https://en.wikipedia.org/wiki/Givens_rotation#Stable_calculation
void FLIPMParamsProvider::Givens_rotation(double a, double b, Eigen::Ref<Eigen::Matrix2d> R, double eps = 1e-10)
{
  double c, s;
  if (fabs(b) < eps)
  {
    c = (a < -eps ? -1 : 1);
    s = 0;
  }
  else if (fabs(a) < eps)
  {
    c = 0;
    s = -sgn(b);
  }
  else if (fabs(a) > fabs(b))
  {
    double t = b / a;
    double u = sgn(a) * fabs(sqrt(1 + t * t));
    c = 1 / u;
    s = -c * t;
  }
  else
  {
    double t = a / b;
    double u = sgn(b) * fabs(sqrt(1 + t * t));
    s = -1 / u;
    c = -s * t;
  }
  R(0, 0) = c, R(0, 1) = -s, R(1, 0) = s, R(1, 1) = c;
}

// The arguments S, T, and Z will be changed.
void FLIPMParamsProvider::swap_block_11(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p)
{
  // Dooren, Case I, p124-125.
  int n2 = static_cast<int>(S.rows());
  Eigen::Matrix2d A = S.block<2, 2>(p, p);
  Eigen::Matrix2d B = T.block<2, 2>(p, p);
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::Matrix2d H = A(1, 1) * B - B(1, 1) * A;
  Givens_rotation(H(0, 1), H(0, 0), Z1.block<2, 2>(p, p));
  S = (S * Z1).eval();
  T = (T * Z1).eval();
  Z = (Z * Z1).eval();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p, p), T(p + 1, p), Q.block<2, 2>(p, p));
  S = (Q * S).eval();
  T = (Q * T).eval();
  S(p + 1, p) = 0;
  T(p + 1, p) = 0;
}

// The arguments S, T, and Z will be changed.
void FLIPMParamsProvider::swap_block_21(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p)
{
  // Dooren, Case II, p126-127.
  int n2 = static_cast<int>(S.rows());
  Eigen::Matrix3d A = S.block<3, 3>(p, p);
  Eigen::Matrix3d B = T.block<3, 3>(p, p);
  // Compute H and eliminate H(1,0) by row operation.
  Eigen::Matrix3d H = A(2, 2) * B - B(2, 2) * A;
  Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);
  Givens_rotation(H(0, 0), H(1, 0), R.block<2, 2>(0, 0));
  H = (R * H).eval();
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  // Compute Z1, Z2, Q1, Q2.
  Givens_rotation(H(1, 2), H(1, 1), Z1.block<2, 2>(p + 1, p + 1));
  H = (H * Z1.block<3, 3>(p, p)).eval();
  Givens_rotation(H(0, 1), H(0, 0), Z2.block<2, 2>(p, p));
  S = (S * Z1).eval();
  T = (T * Z1).eval();
  Z = (Z * Z1 * Z2).eval();
  Givens_rotation(T(p + 1, p + 1), T(p + 2, p + 1), Q1.block<2, 2>(p + 1, p + 1));
  S = (Q1 * S * Z2).eval();
  T = (Q1 * T * Z2).eval();
  Givens_rotation(T(p, p), T(p + 1, p), Q2.block<2, 2>(p, p));
  S = (Q2 * S).eval();
  T = (Q2 * T).eval();
  S(p + 1, p) = 0;
  S(p + 2, p) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
}

// The arguments S, T, and Z will be changed.
void FLIPMParamsProvider::swap_block_12(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p)
{
  int n2 = static_cast<int>(S.rows());
  // Swap the role of S and T.
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q0 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Q3 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(S(p + 1, p + 1), S(p + 2, p + 1), Q0.block<2, 2>(p + 1, p + 1));
  S = (Q0 * S).eval();
  T = (Q0 * T).eval();
  Eigen::Matrix3d A = S.block<3, 3>(p, p);
  Eigen::Matrix3d B = T.block<3, 3>(p, p);
  // Compute H and eliminate H(2,1) by column operation.
  Eigen::Matrix3d H = B(0, 0) * A - A(0, 0) * B;
  Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);
  Givens_rotation(H(2, 2), H(2, 1), R.block<2, 2>(1, 1));
  H = (H * R).eval();
  // Compute Q1, Q2, Z1, Z2.
  Givens_rotation(H(0, 1), H(1, 1), Q1.block<2, 2>(p, p));
  H = (Q1.block<3, 3>(p, p) * H).eval();
  Givens_rotation(H(1, 2), H(2, 2), Q2.block<2, 2>(p + 1, p + 1));
  S = (Q1 * S).eval();
  T = (Q1 * T).eval();
  Givens_rotation(S(p + 1, p + 1), S(p + 1, p), Z1.block<2, 2>(p, p));
  S = (Q2 * S * Z1).eval();
  T = (Q2 * T * Z1).eval();
  Givens_rotation(S(p + 2, p + 2), S(p + 2, p + 1), Z2.block<2, 2>(p + 1, p + 1));
  S = (S * Z2).eval();
  T = (T * Z2).eval();
  Z = (Z * Z1 * Z2).eval();
  // Swap back the role of S and T.
  Givens_rotation(T(p, p), T(p + 1, p), Q3.block<2, 2>(p, p));
  S = (Q3 * S).eval();
  T = (Q3 * T).eval();
  S(p + 2, p) = 0;
  S(p + 2, p + 1) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
}

// The arguments S, T, and Z will be changed.
void FLIPMParamsProvider::swap_block_22(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p)
{
  // Direct Swapping Algorithm based on
  // "Numerical Methods for General and Structured Eigenvalue Problems" by
  // Daniel Kressner, p108-111.
  // ( http://sma.epfl.ch/~anchpcommon/publications/kressner_eigenvalues.pdf ).
  // Also relevant but not applicable here:
  // "On Swapping Diagonal Blocks in Real Schur Form" by Zhaojun Bai and James
  // W. Demmelt;
  int n2 = static_cast<int>(S.rows());
  Eigen::MatrixXd A = S.block<4, 4>(p, p);
  Eigen::MatrixXd B = T.block<4, 4>(p, p);
  // Solve
  // A11 * X - Y A22 = A12
  // B11 * X - Y B22 = B12
  // Reduce to solve Cx=D, where x=[x1;...;x4;y1;...;y4].
  Eigen::Matrix<double, 8, 8> C = Eigen::Matrix<double, 8, 8>::Zero();
  Eigen::Matrix<double, 8, 1> D;
  // clang-format off
  C << A(0, 0), 0, A(0, 1), 0, -A(2, 2), -A(3, 2), 0, 0,
    0, A(0, 0), 0, A(0, 1), -A(2, 3), -A(3, 3), 0, 0,
    A(1, 0), 0, A(1, 1), 0, 0, 0, -A(2, 2), -A(3, 2),
    0, A(1, 0), 0, A(1, 1), 0, 0, -A(2, 3), -A(3, 3),
    B(0, 0), 0, B(0, 1), 0, -B(2, 2), -B(3, 2), 0, 0,
    0, B(0, 0), 0, B(0, 1), -B(2, 3), -B(3, 3), 0, 0,
    B(1, 0), 0, B(1, 1), 0, 0, 0, -B(2, 2), -B(3, 2),
    0, B(1, 0), 0, B(1, 1), 0, 0, -B(2, 3), -B(3, 3);
  // clang-format on
  D << A(0, 2), A(0, 3), A(1, 2), A(1, 3), B(0, 2), B(0, 3), B(1, 2), B(1, 3);
  Eigen::MatrixXd x = C.colPivHouseholderQr().solve(D);
  // Q * [ -Y ] = [ R_Y ] ,  Z' * [ -X ] = [ R_X ] .
  //     [ I  ]   [  0  ]         [ I  ] = [  0  ]
  Eigen::Matrix<double, 4, 2> X, Y;
  X << -x(0, 0), -x(1, 0), -x(2, 0), -x(3, 0), Eigen::Matrix2d::Identity();
  Y << -x(4, 0), -x(5, 0), -x(6, 0), -x(7, 0), Eigen::Matrix2d::Identity();
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::MatrixXd Z1 = Eigen::MatrixXd::Identity(n2, n2);
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 2>> qr1(X);
  Z1.block<4, 4>(p, p) = qr1.householderQ();
  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 2>> qr2(Y);
  Q1.block<4, 4>(p, p) = qr2.householderQ().adjoint();
  // Apply transform Q1 * (S,T) * Z1.
  S = (Q1 * S * Z1).eval();
  T = (Q1 * T * Z1).eval();
  Z = (Z * Z1).eval();
  // Eliminate the T(p+3,p+2) entry.
  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p + 2, p + 2), T(p + 3, p + 2), Q2.block<2, 2>(p + 2, p + 2));
  S = (Q2 * S).eval();
  T = (Q2 * T).eval();
  // Eliminate the T(p+1,p) entry.
  Eigen::MatrixXd Q3 = Eigen::MatrixXd::Identity(n2, n2);
  Givens_rotation(T(p, p), T(p + 1, p), Q3.block<2, 2>(p, p));
  S = (Q3 * S).eval();
  T = (Q3 * T).eval();
  S(p + 2, p) = 0;
  S(p + 2, p + 1) = 0;
  S(p + 3, p) = 0;
  S(p + 3, p + 1) = 0;
  T(p + 1, p) = 0;
  T(p + 2, p) = 0;
  T(p + 2, p + 1) = 0;
  T(p + 3, p) = 0;
  T(p + 3, p + 1) = 0;
  T(p + 3, p + 2) = 0;
}

// Functionality of "swap_block" function:
// swap the 1x1 or 2x2 blocks pointed by p and q.
// There are four cases: swapping 1x1 and 1x1 matrices, swapping 2x2 and 1x1
// matrices, swapping 1x1 and 2x2 matrices, and swapping 2x2 and 2x2 matrices.
// Algorithms are described in the papers
// "A generalized eigenvalue approach for solving Riccati equations" by P. Van
// Dooren, 1981 ( http://epubs.siam.org/doi/pdf/10.1137/0902010 ), and
// "Numerical Methods for General and Structured Eigenvalue Problems" by
// Daniel Kressner, 2005.
void FLIPMParamsProvider::swap_block(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, int p, int q, int q_block_size, double eps = 1e-10)
{
  int p_tmp = q, p_block_size;
  while (p_tmp-- > p)
  {
    p_block_size = 1;
    if (p_tmp >= 1 && fabs(S(p_tmp, p_tmp - 1)) > eps)
    {
      p_block_size = 2;
      p_tmp--;
    }
    switch (p_block_size * 10 + q_block_size)
    {
    case 11:
      swap_block_11(S, T, Z, p_tmp);
      break;
    case 21:
      swap_block_21(S, T, Z, p_tmp);
      break;
    case 12:
      swap_block_12(S, T, Z, p_tmp);
      break;
    case 22:
      swap_block_22(S, T, Z, p_tmp);
      break;
    }
  }
}

// Functionality of "reorder_eigen" function:
// Reorder the eigenvalues of (S,T) such that the top-left n by n matrix has
// stable eigenvalues by multiplying Q's and Z's on the left and the right,
// respectively.
// Stable eigenvalues are inside the unit disk.
//
// Algorithm:
// Go along the diagonals of (S,T) from the top left to the bottom right.
// Once find a stable eigenvalue, push it to top left.
// In implementation, use two pointers, p and q.
// p points to the current block (1x1 or 2x2) and q points to the block with the
// stable eigenvalue(s).
// Push the block pointed by q to the position pointed by p.
// Finish when n stable eigenvalues are placed at the top-left n by n matrix.
// The algorithm for swapping blocks is described in the papers
// "A generalized eigenvalue approach for solving Riccati equations" by P. Van
// Dooren, 1981, and "Numerical Methods for General and Structured Eigenvalue
// Problems" by Daniel Kressner, 2005.
void FLIPMParamsProvider::reorder_eigen(Eigen::Ref<Eigen::MatrixXd> S, Eigen::Ref<Eigen::MatrixXd> T, Eigen::Ref<Eigen::MatrixXd> Z, double eps = 1e-10)
{
  // abs(a) < eps => a = 0
  int n2 = static_cast<int>(S.rows());
  int n = n2 / 2, p = 0, q = 0;

  // Find the first unstable p block.
  while (p < n)
  {
    if (fabs(S(p + 1, p)) < eps)
    { // p block size = 1
      if (fabs(T(p, p)) > eps && fabs(S(p, p)) <= fabs(T(p, p)))
      { // stable
        p++;
        continue;
      }
    }
    else
    { // p block size = 2
      const double det_T = T(p, p) * T(p + 1, p + 1) - T(p + 1, p) * T(p, p + 1);
      if (fabs(det_T) > eps)
      {
        const double det_S = S(p, p) * S(p + 1, p + 1) - S(p + 1, p) * S(p, p + 1);
        if (fabs(det_S) <= fabs(det_T))
        { // stable
          p += 2;
          continue;
        }
      }
    }
    break;
  }
  q = p;

  // Make the first n generalized eigenvalues stable.
  while (p < n && q < n2)
  {
    // Update q.
    int q_block_size = 0;
    while (q < n2)
    {
      if (q == n2 - 1 || fabs(S(q + 1, q)) < eps)
      { // block size = 1
        if (fabs(T(q, q)) > eps && fabs(S(q, q)) <= fabs(T(q, q)))
        {
          q_block_size = 1;
          break;
        }
        q++;
      }
      else
      { // block size = 2
        const double det_T = T(q, q) * T(q + 1, q + 1) - T(q + 1, q) * T(q, q + 1);
        if (fabs(det_T) > eps)
        {
          const double det_S = S(q, q) * S(q + 1, q + 1) - S(q + 1, q) * S(q, q + 1);
          if (fabs(det_S) <= fabs(det_T))
          {
            q_block_size = 2;
            break;
          }
        }
        q += 2;
      }
    }
    if (q >= n2)
      throw std::runtime_error("fail to find enough stable eigenvalues");
    // Swap blocks pointed by p and q.
    if (p != q)
    {
      swap_block(S, T, Z, p, q, q_block_size);
      p += q_block_size;
      q += q_block_size;
    }
  }
  if (p < n && q >= n2)
    throw std::runtime_error("fail to find enough stable eigenvalues");
}

Eigen::MatrixXd FLIPMParamsProvider::solveDARE_analytic(
    const Eigen::Ref<const Eigen::MatrixXd>& A, const Eigen::Ref<const Eigen::MatrixXd>& b, const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R)
{
  // https://github.com/RobotLocomotion/drake/blob/master/math/discrete_algebraic_riccati_equation.cc

  ASSERT((b.cols() <= b.rows()) && "m must be smaller then n");
  const int n = static_cast<int>(b.rows());
  ASSERT((abstol(Q, Q.transpose()) <= 1e-10) && "Q must be symmetric and positive semi-definite");

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Q);
  for (int i = 0; i < n; i++)
  {
    ASSERT((es.eigenvalues()[i] >= -1e-10) && ("Eigenvalue " + std::to_string(i) + " must be positiv!").c_str()); // approx greater or equal zero
  }
  ASSERT((abstol(R, R.transpose()) <= 1e-10) && "R must be symmetric and positive definite");

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  ASSERT((R_cholesky.info() == Eigen::Success) && "Cholesky Decomposition of R failed!");

  //check_stabilizable(A, b);
  //check_detectable(A, Q);

  Eigen::MatrixXd M(2 * n, 2 * n), L(2 * n, 2 * n);
  M << A, Eigen::MatrixXd::Zero(n, n), -Q, Eigen::MatrixXd::Identity(n, n);
  L << Eigen::MatrixXd::Identity(n, n), b * R.inverse() * b.transpose(), Eigen::MatrixXd::Zero(n, n), A.transpose();

  // QZ decomposition of M and L
  // QMZ = S, QLZ = T
  // where Q and Z are real orthogonal matrixes
  // T is upper-triangular matrix, and S is upper quasi-triangular matrix
  Eigen::RealQZ<Eigen::MatrixXd> qz(2 * n);
  qz.compute(M, L); // M = Q S Z,  L = Q T Z (Q and Z computed by Eigen package are adjoints of Q and Z above)
  Eigen::MatrixXd S = qz.matrixS(), T = qz.matrixT(), Z = qz.matrixZ().adjoint();

  // Reorder the generalized eigenvalues of (S,T).
  Eigen::MatrixXd Z2 = Eigen::MatrixXd::Identity(2 * n, 2 * n);
  reorder_eigen(S, T, Z2);
  Z = (Z * Z2).eval();

  // The first n columns of Z is ( U1 ) .
  //                             ( U2 )
  //            -1
  // X = U2 * U1   is a solution of the discrete time Riccati equation.
  Eigen::MatrixXd U1 = Z.block(0, 0, n, n), U2 = Z.block(n, 0, n, n);
  Eigen::MatrixXd X = U2 * U1.inverse();
  X = (X + X.adjoint().eval()) / 2.0;
  return X;
}

MAKE_MODULE(FLIPMParamsProvider, dortmundWalkingEngine)
