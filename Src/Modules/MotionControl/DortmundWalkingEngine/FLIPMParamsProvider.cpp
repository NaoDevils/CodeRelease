#include "FLIPMParamsProvider.h"
#include "Tools/Module/ModuleManager.h"
#include <iostream>

FLIPMParamsProvider::FLIPMParamsProvider() {
  initializedX = true;
  initializedY = true;
  calculationValidX = true;
  calculationValidY = true;

  xLQRParams.clear();
  yLQRParams.clear();

  last_S[0] = paramsX.Ql.cast<double>();
  last_S[1] = paramsY.Ql.cast<double>();

  param_mutex_X.lock();
    finishedCalculationX = false;
    threadXStarted = false;
    sharedResultsXLQRParams.clear();
  param_mutex_X.unlock();
  calculationThreadX.setPriority(threadPriority);

  param_mutex_Y.lock();
    finishedCalculationY = false;
    threadYStarted = false;
    sharedResultsYLQRParams.clear();
  param_mutex_Y.unlock();
  calculationThreadY.setPriority(threadPriority);

  FLIPMControllerParameter loadedFLIPMControllerParameter;
  InMapFile fileC("flipmControllerParameter.cfg");
  if (fileC.exists())
    fileC >> loadedFLIPMControllerParameter;
  else
  {
    ASSERT(false);
  }

  FLIPMObserverParameter loadedFLIPMObserverParameter;
  InMapFile fileO("flipmObserverParameter.cfg");
  if (fileO.exists())
    fileO >> loadedFLIPMObserverParameter;
  else
  {
    ASSERT(false);
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
}

bool FLIPMParamsProvider::checkObservability(const Matrix6d &A, const Matrix3x6d &c) const {
  // calculate observability matrix
  Eigen::Matrix<double, 18, 6> obsv;

  obsv << c,
          c * A,
          c * (A * A),
          c * (A * A * A),
          c * (A * A * A * A),
          c * (A * A * A * A * A);

  Eigen::FullPivLU<Eigen::Matrix<double, 18, 6>> lu_decomp(obsv);
  lu_decomp.setThreshold(1e-10);
  int rank = static_cast<int>(lu_decomp.rank());

  if (rank == A.rows()) {
    return true;
  }
  else {
    return false;
  }
}

bool FLIPMParamsProvider::checkControllability(const Matrix7d &A, const Vector7d &b) const {
  // calculate controllability matrix
  Matrix7d ctrb;

  ctrb << b, A * b, (A * A) * b, (A * A * A) * b, (A * A * A * A) * b, (A * A * A * A * A) * b, (A * A * A * A * A * A) * b;

  Eigen::FullPivLU<Matrix7d> lu_decomp(ctrb);
  lu_decomp.setThreshold(1e-10);
  int rank = static_cast<int>(lu_decomp.rank());

  if (rank == A.rows()) {
    return true;
  }
  else {
    return false;
  }
}

void FLIPMParamsProvider::update(FLIPMControllerParameter& flipmControllerParameter)
{
  DEBUG_RESPONSE_ONCE("module:FLIPMParamsProvider:recalculate") {
    initializedX = false;
    initializedY = false;
    calculationValidX = false;
    calculationValidY = false;
  }

  param_mutex_X.lock();
    if (finishedCalculationX) {
      xLQRParams = sharedResultsXLQRParams;
      finishedCalculationX = false;
      threadXStarted = false;
      SystemCall::playSound("allright.wav");
      //OUTPUT_TEXT("Recalculated X-Params");
    }
  param_mutex_X.unlock();

  param_mutex_Y.lock();
    if (finishedCalculationY) {
      yLQRParams = sharedResultsYLQRParams;
      finishedCalculationY = false;
      threadYStarted = false;
      SystemCall::playSound("allright.wav");
      //OUTPUT_TEXT("Recalculated Y-Params");
    }
  param_mutex_Y.unlock();

  if (!initializedX && !threadXStarted) {
    param_mutex_X.lock();
      threadXStarted = true;
    param_mutex_X.unlock();
    calculationThreadX.start(this, &FLIPMParamsProvider::executeX);
    //execute(X);
  }

  if (!duplicateXParams && !initializedY && !threadYStarted) {
    param_mutex_Y.lock();
      threadYStarted = true;
    param_mutex_Y.unlock();
    calculationThreadY.start(this, &FLIPMParamsProvider::executeY);
    //execute(Y);
  }


  if (calculationValidX) {
    flipmControllerParameter.controllerParamsX.A = xLQRParams.A;
    flipmControllerParameter.controllerParamsX.b = xLQRParams.b;
    flipmControllerParameter.controllerParamsX.c = xLQRParams.c;
    flipmControllerParameter.controllerParamsX.Gi = xLQRParams.Gi;
    flipmControllerParameter.controllerParamsX.Gd = xLQRParams.Gd;
    flipmControllerParameter.controllerParamsX.Gx = xLQRParams.Gx;
  }

  if (!duplicateXParams && calculationValidY) {
    flipmControllerParameter.controllerParamsY.A = yLQRParams.A;
    flipmControllerParameter.controllerParamsY.b = yLQRParams.b;
    flipmControllerParameter.controllerParamsY.c = yLQRParams.c;
    flipmControllerParameter.controllerParamsY.Gi = yLQRParams.Gi;
    flipmControllerParameter.controllerParamsY.Gd = yLQRParams.Gd;
    flipmControllerParameter.controllerParamsY.Gx = yLQRParams.Gx;
  }
  else if (calculationValidX) {
    flipmControllerParameter.controllerParamsY.A = xLQRParams.A;
    flipmControllerParameter.controllerParamsY.b = xLQRParams.b;
    flipmControllerParameter.controllerParamsY.c = xLQRParams.c;
    flipmControllerParameter.controllerParamsY.Gi = xLQRParams.Gi;
    flipmControllerParameter.controllerParamsY.Gd = xLQRParams.Gd;
    flipmControllerParameter.controllerParamsY.Gx = xLQRParams.Gx;
  }
}

void FLIPMParamsProvider::update(FLIPMObserverParameter& flipmObserverParameter)
{
  DEBUG_RESPONSE_ONCE("module:FLIPMParamsProvider:recalculate") {
    initializedX = false;
    initializedY = false;
    calculationValidX = false;
    calculationValidY = false;
  }
  param_mutex_X.lock();
  if (finishedCalculationX) {
    xLQRParams = sharedResultsXLQRParams;
    finishedCalculationX = false;
    threadXStarted = false;
    OUTPUT_TEXT("Recalculated X-Params");
  }
  param_mutex_X.unlock();

  param_mutex_Y.lock();
  if (finishedCalculationY) {
    yLQRParams = sharedResultsYLQRParams;
    finishedCalculationY = false;
    threadYStarted = false;
    OUTPUT_TEXT("Recalculated Y-Params");
  }
  param_mutex_Y.unlock();

  if (!initializedX && !threadXStarted) {
    param_mutex_X.lock();
    threadXStarted = true;
    param_mutex_X.unlock();
    calculationThreadX.start(this, &FLIPMParamsProvider::executeX);
    //execute(X);
  }

  if (!duplicateXParams && !initializedY && !threadYStarted) {
    param_mutex_Y.lock();
    threadYStarted = true;
    param_mutex_Y.unlock();
    calculationThreadY.start(this, &FLIPMParamsProvider::executeY);
    //execute(Y);
  }

  if (calculationValidX) {
    flipmObserverParameter.observerParamsX.L = xLQRParams.L;
  } 

  if (!duplicateXParams && calculationValidY)
    flipmObserverParameter.observerParamsY.L = yLQRParams.L;
  else if (calculationValidX)
    flipmObserverParameter.observerParamsY.L = xLQRParams.L;
}

void FLIPMParamsProvider::execute(Dimension dim) 
{
  if (dim == Dimension::X) 
    calculationValidX = true;
  else
    calculationValidY = true;
  
  std::mutex &param_mutex = dim == Dimension::X ? param_mutex_X : param_mutex_Y;
  const FLIPMValues &params = dim == Dimension::X ? paramsX : paramsY;
  bool &finishedCalculation = dim == Dimension::X ? finishedCalculationX : finishedCalculationY;
  
  param_mutex.lock();
    finishedCalculation = false;
  param_mutex.unlock();

  LQRParams lqrParams;
  lqrParams.clear();

  double dt = params.dt;
  double dt2 = (dt * dt) / 2;
  double dt3 = (dt * dt * dt) / 6;
  double z_h = params.z_h;
  double g = params.g;

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
  double R = params.R;
  
  Matrix6d Ql = params.Ql.cast<double>();
  Matrix3d RO = params.RO.cast<double>();
    
  ////////////////////////////////////////////////////////////////////
  //lqrParams.A = Matrix6d::Zero();
  lqrParams.A <<  1,          dt,       dt2,  0,          0,            0,
                  0,          1,        dt,   0,          0,            0,
                 -DM,        -EM,       0,    DM,         EM,           0,
                  0,          0,        0,    1,          dt,           dt2,
                  Dm * dt,    Em * dt,  0,   -Dm * dt,    1 - Em * dt,  dt,
                  0,          0,        0,    0,          0,            1;

  //lqrParams.b = Vector6d::Zero();
  lqrParams.b << 0, 0, 0, dt3, dt2, dt;

  //lqrParams.c = Matrix1x6d::Zero();
  lqrParams.c << 1, 0, -z_h / g, 0, 0, 0;

  Vector7d Bt = Vector7d::Zero();
  Bt << lqrParams.c.dot(lqrParams.b) , lqrParams.b;

  Vector7d It = Vector7d::Zero();
  It(0,0) = 1;

  Matrix7x6d Ft = Matrix7x6d::Zero();
  Ft << lqrParams.c*lqrParams.A, lqrParams.A;

  Matrix7d Qt = Matrix7d::Zero();
  Vector6d cT = lqrParams.c.transpose();
  Vector6d cTQx = cT * Qx;
  Matrix6d cTQxC = cTQx * lqrParams.c;
  Qt(0, 0) = Qe;
  Qt.block(1, 1, 6, 6) = cTQxC;

  Matrix7d At = Matrix7d::Zero();
  At.block(0, 0, 7, 1) = It;
  At.block(0, 1, 7, 6) = Ft;

  Eigen::Matrix<double, 1, 1> Rt = Eigen::Matrix<double, 1, 1>::Ones();
  Rt << R;

  if (checkControllability(At, Bt)) {
    Matrix7d Pt = solveDARE<7,1>(At, Bt, Qt, Rt, Qt, DARE_maxIterations, 1e-8f);

    //lqrParams.Gx = Vector6d::Zero();
    lqrParams.Gx = (1.0 / Eigen::Matrix<double, 1, 1>(R + Bt.transpose() * Pt * Bt)(0)) * (Bt.transpose() * Pt * Ft);

    //lqrParams.Gi = 0.f;
    lqrParams.Gi = ((1.0 / Eigen::Matrix<double, 1, 1>(R + Bt.transpose() * Pt * Bt)(0)) * (Bt.transpose() * Pt * It))(0);

    Matrix7d Ac = Matrix7d::Zero();
    Ac = At - Bt * (1.0 / Eigen::Matrix<double, 1, 1>(R + Bt.transpose() * Pt * Bt)(0)) * (Bt.transpose() * Pt * At);

    Vector7d X = Vector7d::Zero();
    X = -Ac.transpose() * Pt * It;

    std::vector<double> Gd_vec;
    Gd_vec.reserve(PREVIEW_LENGTH);
    Gd_vec.push_back(-lqrParams.Gi);
    for (int i = 1; i < PREVIEW_LENGTH; i++) {
      Gd_vec.push_back((1 / Eigen::Matrix<double, 1, 1>(R + Bt.transpose() * Pt * Bt)(0)) * Bt.transpose() * X);
      X = Ac.transpose() * X;
    }
    Eigen::Matrix<double, PREVIEW_LENGTH, 1> Gd(Gd_vec.data());
    lqrParams.Gd = Gd;

    // OLD Observable States
    Matrix3x6d Cm = Matrix3x6d::Zero();
    Cm(0, 0) = 1;
    Cm(1, 2) = 1;
    Cm(2, 3) = 1;

    //// NEW Observable States
    //Matrix3x6d Cm = Matrix3x6d::Zero();
    //Cm(0, 0) = 1;
    //Cm(1, 1) = 1;
    //Cm(2, 2) = 1;

    if (checkObservability(lqrParams.A, Cm)) {
      Matrix6x3d desiredL = paramsFLIPMContY.L;

      const unsigned long long startTime = SystemCall::getCurrentThreadTime();
      Matrix6x3d resultingL = dlqr(lqrParams.A.transpose(), Cm.transpose(), Ql, RO, dim);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> resultingL_rowmajor(resultingL);
      Eigen::Map<Matrix6x3d> L(resultingL_rowmajor.data(), 6,3);
      lqrParams.L = L;
      const unsigned long long stopTime = SystemCall::getCurrentThreadTime();

      const unsigned diff_time2 = unsigned(stopTime - startTime);
      Matrix6x3d percentualDiff = (desiredL - lqrParams.L).cwiseQuotient(desiredL);
      double diff2 = percentualDiff.cwiseAbs().maxCoeff();
      //param_mutex.lock();
      std::cout << "solveDARE took: " << (diff_time2 / 1000) << "ms | diff: " << diff2 * 100 << "%" << std::endl;
      //param_mutex.unlock();
    }
    else {
      if (dim == Dimension::X)
        calculationValidX = false;
      else
        calculationValidY = false;
      //param_mutex.lock();
      std::cout << "Observability is not given!!! Try to change parameters..." << std::endl;
      //param_mutex.unlock();
    }
  }
  else {
    if (dim == Dimension::X)
      calculationValidX = false;
    else
      calculationValidY = false;
    //param_mutex.lock();
    std::cout << "Controllability is not given!!! Try to change parameters..." << std::endl;
    //param_mutex.unlock();
  }
  
  if (dim == Dimension::X)
    initializedX = true;
  else
    initializedY = true;

  param_mutex.lock();
    finishedCalculation = true;
    LQRParams &sharedResults = dim == Dimension::X ? sharedResultsXLQRParams : sharedResultsYLQRParams;
    sharedResults = lqrParams;
  param_mutex.unlock();
}

Matrix6x3d FLIPMParamsProvider::dlqr(const Matrix6d &A, const Matrix6x3d &b, const Matrix6d &Q, const Matrix3d &R, const Dimension &dim) {
  Matrix6x3d L = Matrix6x3d::Zero();
  Matrix3x6d bt = b.transpose();
  Matrix6d S = solveDARE<6,3>(A,b,Q,R,last_S[dim],DARE_maxIterations,DARE_threshold);
  last_S[dim] = S;
  Matrix3x6d btS = bt * S;
  Matrix3x6d btSA = btS * A;
  Matrix3d btSb = btS * b;
  Matrix3d btSbR = R + btSb ;
  Matrix3d btSbR_inv = btSbR.inverse();

  Matrix3x6d tempL = btSbR_inv * btSA;
  L = tempL.transpose();
  return L;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
Eigen::Matrix<double, STATE_DIM, STATE_DIM> FLIPMParamsProvider::solveDARE_scipy(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R) {
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
  
  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM> H = Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM>::Zero();
  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM> J = Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM>::Zero();

  H.block(0,            0,                STATE_DIM,    STATE_DIM)    << A;
  H.block(0,            2*STATE_DIM,      STATE_DIM,    CONTROL_DIM)  << b;
  H.block(STATE_DIM,    0,                STATE_DIM,    STATE_DIM)    << -Q;
  H.block(STATE_DIM,    STATE_DIM,        STATE_DIM,    STATE_DIM)    << Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity(STATE_DIM, STATE_DIM);
  H.block(2*STATE_DIM,  2*STATE_DIM,      CONTROL_DIM,  CONTROL_DIM)  << R;

  J.block(0,            0,                STATE_DIM,    STATE_DIM) << Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity(STATE_DIM, STATE_DIM);
  J.block(STATE_DIM,    STATE_DIM,        STATE_DIM,    STATE_DIM) << A.transpose();
  J.block(2*STATE_DIM,  STATE_DIM,        CONTROL_DIM,  STATE_DIM) << -b.transpose();

  // TODO Balance

  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, CONTROL_DIM> H_lastCols = H.block(0, 2 * STATE_DIM + CONTROL_DIM - CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM, CONTROL_DIM);
  Eigen::HouseholderQR<MatrixXd> qr(H_lastCols);
  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM> q_of_qr = qr.householderQ();

  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM> H_firstCols = H.block(0, 0, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM);
  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM> J_firstCols = J.block(0, 0, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM);
  Eigen::Matrix<double, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM> q_of_qr_lastCols = q_of_qr.block(0, CONTROL_DIM, 2 * STATE_DIM + CONTROL_DIM, 2 * STATE_DIM);

  Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> H2 = q_of_qr_lastCols.transpose() * H_firstCols;
  Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> J2 = q_of_qr_lastCols.transpose() * J_firstCols;

  Eigen::RealQZ<Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM>> qz(2 * STATE_DIM);
  qz.compute(H2, J2);
  Eigen::Matrix<double, 2 * STATE_DIM, 2 * STATE_DIM> u = qz.matrixZ();
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> u00 = u.block(0, 0, STATE_DIM, STATE_DIM);
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> u01 = u.block(STATE_DIM, STATE_DIM, STATE_DIM, STATE_DIM);

  //Eigen::FullPivLU<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> lu(u00);
  //Eigen::Matrix<double, STATE_DIM, STATE_DIM> up = lu.permutationP();
  //Eigen::Matrix<double, STATE_DIM, STATE_DIM> ul = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
  //ul.block<STATE_DIM, STATE_DIM>(0, 0).triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
  //Eigen::Matrix<double, STATE_DIM, STATE_DIM> uu = lu.matrixLU().triangularView<Eigen::Upper>();


  //std::cout << H << std::endl;
  return P;
}

template <size_t STATE_DIM, size_t CONTROL_DIM>
Eigen::Matrix<double, STATE_DIM, STATE_DIM> FLIPMParamsProvider::solveDARE(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &initialP, unsigned int maxIterations, float threshold) {
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P = initialP;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_last;
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> K = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>::Zero();

  //double P_norm_old = std::numeric_limits<double>::max();
  bool finishedRecursion = false;
  Eigen::EigenSolver<Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>> eigenvalueSolver;

  size_t i = 0;
  while (!finishedRecursion) {
    P_last = P; 

    if(useRobustIterationMethod)
      iterateRobust<STATE_DIM,CONTROL_DIM>(A, b, Q, R, P, K, eigenvalueSolver);
    else
      iterateNaive<STATE_DIM,CONTROL_DIM>(A, b, Q, R, P, K);

    double diff = (P - P_last).cwiseQuotient(P).cwiseAbs().maxCoeff();
    //double P_norm = P.norm();
    //double diff = std::abs(P_norm - P_norm_old);
    if (diff < threshold || i > maxIterations)
    {
      finishedRecursion = true;
      std::cout << "DARE ended with " << i << " Iterations and a diff of " << diff << std::endl;
    }
    //P_norm_old = P_norm;
    i++;
  }

  return P;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copied from https://adrlab.bitbucket.io/ct/v2.1/ct_optcon/doc/html/DynamicRiccatiEquation_8hpp_source.html ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Naive, direct literal implementation
// computes P = Q + A^T ( P - P B ( R + B^T P B )^-1 B^T P) A.
//            = Q + A^T ( P - P B H_inverse B^T P) A
//            = Q + A^T S A
// where H_inverse = (R + B^T P B)^-1 and S = P - P B H_inverse B^T * P
// i.e. P is altered and H_inverse is returned for convenience and use in other functions
template <size_t STATE_DIM, size_t CONTROL_DIM>
void FLIPMParamsProvider::iterateNaive(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P, Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> &K)
{
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> H = R;
  H.noalias() += b.transpose() * P * b;
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> H_inverse = H.inverse();

  K.noalias() = -1.0 * H_inverse * b.transpose() * P * A;

  // here we compute P
  P = Q + (A.transpose() * P * A);
  P.noalias() -= K.transpose() * H * K;

  P = (P + P.transpose()).eval() / 2.0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copied from https://adrlab.bitbucket.io/ct/v2.1/ct_optcon/doc/html/DynamicRiccatiEquation_8hpp_source.html ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <size_t STATE_DIM, size_t CONTROL_DIM>
void FLIPMParamsProvider::iterateRobust(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P, Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> &K, Eigen::EigenSolver<Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>> &eigenvalueSolver)
{
  double epsilon = 1e-4;
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> H = R;
  H.noalias() += b.transpose() * P * b;
  H = (H + H.transpose()).eval() / 2.0;

  // compute eigenvalues with eigenvectors enabled
  eigenvalueSolver.compute(H, true);
  const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &V = eigenvalueSolver.eigenvectors().real();
  const Eigen::Matrix<double, CONTROL_DIM, 1> &lambda = eigenvalueSolver.eigenvalues().real();
  ASSERT(eigenvalueSolver.eigenvectors().imag().norm() < 1e-7 && "Eigenvectors not real");
  ASSERT(eigenvalueSolver.eigenvalues().imag().norm() < 1e-7 && "Eigenvalues is not real");
  // Corrected Eigenvalue Matrix
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> D = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>::Zero();
  // make D positive semi-definite (as described in IV. B.)
  D.diagonal() = lambda.cwiseMax(Eigen::Matrix<double, CONTROL_DIM, 1>::Zero()) + epsilon * Eigen::Matrix<double, CONTROL_DIM, 1>::Ones();

  ASSERT((V.transpose() - V.inverse()).norm() < epsilon && "WARNING: V transpose and V inverse are not similar!");

  // reconstruct H
  H.noalias() = V * D * V.transpose();

  // invert D
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> D_inverse = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>::Zero();
  // eigenvalue-wise inversion
  D_inverse.diagonal() = D.diagonal().cwiseInverse();
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> H_inverse = V * D_inverse * V.transpose();

  K.noalias() = -1.0 * H_inverse * (b.transpose() * P * A);

  // here we compute P
  P = Q + (A.transpose() * P * A);
  P.noalias() -= K.transpose() * H * K;

  P = (P + P.transpose()).eval() / 2.0;
}

MAKE_MODULE(FLIPMParamsProvider, dortmundWalkingEngine)