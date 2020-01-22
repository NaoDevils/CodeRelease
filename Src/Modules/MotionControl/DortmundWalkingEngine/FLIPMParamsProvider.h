#pragma once

#include "Representations/MotionControl/FLIPMParams.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Streams/RobotParameters.h"
#include "Tools/Module/Module.h"

#include "Platform/Thread.h"
#include <mutex>

MODULE(FLIPMParamsProvider,
{ ,
  PROVIDES_WITHOUT_MODIFY(FLIPMParameter),
  PROVIDES(FLIPMControllerParameter),
  PROVIDES(FLIPMObserverParameter),

  LOADS_PARAMETERS(
  { ,
    (bool)(false) useRCS,
    (bool)(true) duplicateXParams,
    (unsigned)(800000) DARE_maxIterations,
    (float)(0.000001) DARE_threshold,
    (int)(0) threadPriority,
    (bool) useRobustIterationMethod,
    (FLIPMValues) paramsX,
    (FLIPMValues) paramsY,
  }),
});

class FLIPMParamsProvider : public FLIPMParamsProviderBase
{
  // Params for Sanity Check 
  ROBOT_PARAMETER_CLASS(FLIPMContY, FLIPMParamsProvider)
    //PARAM(Matrix6d, A)
    //PARAM(Vector6d, b)
    //PARAM(Matrix1x6d, c)
    //PARAM(double, Gi)
    //PARAM(Matrix1x6d, Gx)
    //PARAM(Vector50d, Gd)
    PARAM(Matrix6x3d, L)
  END_ROBOT_PARAMETER_CLASS(FLIPMContY)

public:
  FLIPMParamsProvider();

  class LQRParams
  {
    public:
      LQRParams() {
        clear();
      }
  
      Matrix6d A;
      Vector6d b;
      Matrix1x6d c;
      double Gi;
      Matrix1x6d Gx;
      Eigen::Matrix<double, PREVIEW_LENGTH, 1> Gd;
      Matrix6x3d L;

      void clear() {
        A = Matrix6d::Zero();
        b = Vector6d::Zero();
        c = Matrix1x6d::Zero();
        Gi = 0.0;
        Gx = Matrix1x6d::Zero();
        Gd = Eigen::Matrix<double, PREVIEW_LENGTH, 1>::Zero();
        L = Matrix6x3d::Zero();
      }
  };

protected:
  ENUM(Dimension,
  { ,
    X,
    Y,
  });

  bool initializedX = false;
  bool initializedY = false;
  bool calculationValidX = true;
  bool calculationValidY = true;
  LQRParams xLQRParams;
  LQRParams yLQRParams;

  Matrix6d last_S[2];

  Thread<FLIPMParamsProvider> calculationThreadX, calculationThreadY;
  std::mutex param_mutex_X, param_mutex_Y;
  LQRParams sharedResultsXLQRParams, sharedResultsYLQRParams;
  bool finishedCalculationX, finishedCalculationY;
  bool threadXStarted, threadYStarted;

  bool initializedFLIPMParams = false;
  FLIPMParameter lastFLIPMParams;

  void update(FLIPMParameter& flipmParameter) {
    flipmParameter.paramsX = paramsX;
    flipmParameter.paramsY = paramsY;
  }
  void update(FLIPMControllerParameter& flipmControllerParameter);
  void update(FLIPMObserverParameter& flipmObserverParameter);
  void execute(Dimension dim);
  void executeX() { execute(X); }
  void executeY() { execute(Y); }
  bool checkObservability(const Matrix6d &A, const Matrix3x6d &c) const;
  bool checkControllability(const Matrix7d &A, const Vector7d &b) const;

  template <size_t STATE_DIM, size_t CONTROL_DIM>
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> solveDARE(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &initialP, unsigned int maxIterations, float threshold);
  
  template <size_t STATE_DIM, size_t CONTROL_DIM>
  void iterateNaive(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, Eigen::Matrix<double, STATE_DIM, STATE_DIM> &P, Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> &K);
 
  template <size_t STATE_DIM, size_t CONTROL_DIM>
  void iterateRobust(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P, Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>& K, Eigen::EigenSolver<Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>> &eigenvalueSolver);

  template <size_t STATE_DIM, size_t CONTROL_DIM>
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> solveDARE_scipy(const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &A, const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> &b, const Eigen::Matrix<double, STATE_DIM, STATE_DIM> &Q, const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> &R);

  Matrix6x3d dlqr(const Matrix6d &A, const Matrix6x3d &b, const Matrix6d &Q, const Matrix3d &R, const Dimension &dim);
};




