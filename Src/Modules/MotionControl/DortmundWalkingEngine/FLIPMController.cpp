#include "FLIPMController.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

FLIPMController::FLIPMController()
{
  reset();
}

void FLIPMController::reset()
{
  dynamic_z_h = max_z_h;
  x.x() = Eigen::Matrix<double, 6, 1>::Zero();
  x.y() = Eigen::Matrix<double, 6, 1>::Zero();
  v = Vector2d::Zero();

  x_RCS.x() = Eigen::Matrix<double, 6, 1>::Zero();
  x_RCS.y() = Eigen::Matrix<double, 6, 1>::Zero();
  v_RCS = Vector2d::Zero();

  isRunning = false;
  framesToInterpolate = 0;
}

void FLIPMController::executeController(Dimension d, const Eigen::Matrix<double, 1, PREVIEW_LENGTH>& refZMP)
{
  Eigen::Matrix<double, 6, 1> err = theObservedFLIPMError.ObservedError[d]; //+ theReferenceModificator.handledErr[d];

  /*
    Eigen::Matrix< float, 6, 1> offsetFromParameters = Eigen::Matrix<float, 6, 1>::Zero();
    offsetFromParameters(0, 0) = theWalkingEngineParams.comOffsets.xFixed;
    float yOffset = theWalkingEngineParams.comOffsets.ySpeedDependent[theFootpositions.speed.y < 0] * 1000.0f * std::fabs(theFootpositions.speed.y * 1000.0f) / theWalkingEngineParams.speedLimits.y;
    offsetFromParameters(1, 0) = theWalkingEngineParams.comOffsets.yFixed + yOffset;
    */

  if (d == X)
  {
    double u1 = theFLIPMControllerParameter.controllerParamsX.Gi * v[d];
    double u2 = theFLIPMControllerParameter.controllerParamsX.Gx * x[d];
    double u3 = refZMP * theFLIPMControllerParameter.controllerParamsX.Gd;
    double u = -u1 - u2 - u3;
    Eigen::Matrix<double, 6, 1> x1 = theFLIPMControllerParameter.controllerParamsX.A * x[d];
    Eigen::Matrix<double, 6, 1> x2 = theFLIPMControllerParameter.controllerParamsX.b * u;
    x[d] = x1 + x2 + err; // +offsetFromParameters;
    double v1 = theFLIPMControllerParameter.controllerParamsX.c * x[d];
    double v2 = refZMP[0];
    v[d] += v1 - v2;
  }
  else
  {
    double u1 = theFLIPMControllerParameter.controllerParamsY.Gi * v[d];
    double u2 = theFLIPMControllerParameter.controllerParamsY.Gx * x[d];
    double u3 = refZMP * theFLIPMControllerParameter.controllerParamsY.Gd;
    double u = -u1 - u2 - u3;
    Eigen::Matrix<double, 6, 1> x1 = theFLIPMControllerParameter.controllerParamsY.A * x[d];
    Eigen::Matrix<double, 6, 1> x2 = theFLIPMControllerParameter.controllerParamsY.b * u;
    x[d] = x1 + x2 + err; // +offsetFromParameters;
    double v1 = theFLIPMControllerParameter.controllerParamsY.c * x[d];
    double v2 = refZMP[0];
    v[d] += v1 - v2;
  }
}

void FLIPMController::executeRCSController(Dimension d, const Eigen::Matrix<double, 1, PREVIEW_LENGTH>& refZMP)
{
  Eigen::Matrix<double, 6, 1> err = theObservedFLIPMError.ObservedError[d]; //+ theReferenceModificator.handledErr[d];

  if (d == X)
  {
    double u1 = theFLIPMControllerParameter.controllerParamsX.Gi * v_RCS[d];
    double u2 = theFLIPMControllerParameter.controllerParamsX.Gx * x_RCS[d];
    double u3 = refZMP * theFLIPMControllerParameter.controllerParamsX.Gd;
    double u = -u1 - u2 - u3;
    Eigen::Matrix<double, 6, 1> x1 = theFLIPMControllerParameter.controllerParamsX.A * x_RCS[d];
    Eigen::Matrix<double, 6, 1> x2 = theFLIPMControllerParameter.controllerParamsX.b * u;
    x_RCS[d] = x1 + x2 + err;
    double v1 = theFLIPMControllerParameter.controllerParamsX.c * x_RCS[d];
    double v2 = refZMP[0];
    v_RCS[d] += v1 - v2;
  }
  else
  {
    double u1 = theFLIPMControllerParameter.controllerParamsY.Gi * v_RCS[d];
    double u2 = theFLIPMControllerParameter.controllerParamsY.Gx * x_RCS[d];
    double u3 = refZMP * theFLIPMControllerParameter.controllerParamsY.Gd;
    double u = -u1 - u2 - u3;
    Eigen::Matrix<double, 6, 1> x1 = theFLIPMControllerParameter.controllerParamsY.A * x_RCS[d];
    Eigen::Matrix<double, 6, 1> x2 = theFLIPMControllerParameter.controllerParamsY.b * u;
    x_RCS[d] = x1 + x2 + err;
    double v1 = theFLIPMControllerParameter.controllerParamsY.c * x_RCS[d];
    double v2 = refZMP[0];
    v_RCS[d] += v1 - v2;
  }
}

Point FLIPMController::controllerStep()
{
  if (theRefZMP2018.reset)
  {
#ifdef TARGET_ROBOT
#pragma unroll
#endif
    for (int i = 0; i < PREVIEW_LENGTH; i++)
    {
      lastRefZMP(X)(0, i) = refZMP(X)(0, i);
      lastRefZMP(Y)(0, i) = refZMP(Y)(0, i);
    }
  }
#ifdef TARGET_ROBOT
#pragma unroll
#endif

  for (int i = 0; i < PREVIEW_LENGTH; i++)
  {
    if (i < PREVIEW_LENGTH - 1)
    {
      PLOT("module:FLIPMController:refZMPDiff.x", theRefZMP2018.zmpWCS[i].x() - refZMP(X)(0, i + 1));
      PLOT("module:FLIPMController:reset", theRefZMP2018.reset ? 0.01f : 0.f);
    }
    refZMP(X)(0, i) = theRefZMP2018.zmpWCS[i].x();
    refZMP(Y)(0, i) = theRefZMP2018.zmpWCS[i].y();
  }

  if (theFLIPMControllerParameter.useRefZMPInterpolation && (theRefZMP2018.reset || framesToInterpolate > 0))
  {
    if (theRefZMP2018.reset)
    {
      framesToInterpolate = theFLIPMControllerParameter.framesToInterpolate;
    }

    for (int i = 0; i < framesToInterpolate; i++)
    {
      double ratio = static_cast<double>(i + 1 + (theFLIPMControllerParameter.framesToInterpolate - framesToInterpolate)) / static_cast<double>(theFLIPMControllerParameter.framesToInterpolate);
      refZMP(X)(0, i) = lastRefZMP(X)(0, i + 1 + (theFLIPMControllerParameter.framesToInterpolate - framesToInterpolate)) * (1.0 - ratio) + refZMP(X)(0, i) * ratio;
      refZMP(Y)(0, i) = lastRefZMP(Y)(0, i + 1 + (theFLIPMControllerParameter.framesToInterpolate - framesToInterpolate)) * (1.0 - ratio) + refZMP(Y)(0, i) * ratio;
    }
    framesToInterpolate--;
  }

  PLOT("module:FLIPMController:refZMP.x", refZMP(X)(0, 0));
  PLOT("module:FLIPMController:refZMP.y", refZMP(Y)(0, 0));

  executeController(X, refZMP.x());
  executeController(Y, refZMP.y());


  Eigen::Matrix<Eigen::Matrix<double, 1, PREVIEW_LENGTH>, 2, 1> refZMP_RCS;

  //Rensen: added because of warning above
#ifdef TARGET_ROBOT
#pragma unroll
#endif
  for (int i = 0; i < PREVIEW_LENGTH; i++)
  {
    refZMP_RCS(X)(0, i) = theRefZMP2018.zmpRCS[i].x();
    refZMP_RCS(Y)(0, i) = theRefZMP2018.zmpRCS[i].y();
  }

  PLOT("module:FLIPMController:refZMP_RCS.x", theRefZMP2018.zmpRCS[0].x());
  PLOT("module:FLIPMController:refZMP_RCS.y", theRefZMP2018.zmpRCS[0].y());

  executeRCSController(X, refZMP_RCS.x());
  executeRCSController(Y, refZMP_RCS.y());

  Pose2f robotPosition(theWalkingInfo.robotPosition.rotation, Vector2f(theWalkingInfo.robotPosition.translation.x(), theWalkingInfo.robotPosition.translation.y()));
  Point targetCOM(static_cast<float>(x_RCS(X)(3, 0)), static_cast<float>(x_RCS(Y)(3, 0)));

  //float direction = kElement_RCS->direction;
  //targetCOM.rotate2D(direction);

  PLOT("module:FLIPMController:targetCoM_RCS.x", targetCOM.x);
  PLOT("module:FLIPMController:targetCoM_RCS.y", targetCOM.y);
  PLOT("module:FLIPMController:targetCoM.x", x(X)(3, 0));
  PLOT("module:FLIPMController:targetCoM.y", x(Y)(3, 0));
  PLOT("module:FLIPMController:targetCoMDiff.x", x(X)(3, 0) - targetCOM.x);
  PLOT("module:FLIPMController:targetCoMDiff.y", x(Y)(3, 0) - targetCOM.y);

  float yLimit = (theWalkingEngineParams.speedLimits.y * theWalkingEngineParams.speedLimits.speedFactor) / 1000.f;
  float factorY = std::abs(yLimit - std::abs(theSpeedInfo.speed.translation.y())) / yLimit;
  float desired_z_h = factorY * max_z_h + (1.f - factorY) * min_z_h;
  float diffToDynamic = desired_z_h - dynamic_z_h;

  if (diffToDynamic > epsilon_z_h)
  {
    diffToDynamic = epsilon_z_h;
  }
  else if (diffToDynamic < -epsilon_z_h)
  {
    diffToDynamic = -epsilon_z_h;
  }
  else
  {
    diffToDynamic = 0.f;
  }

  dynamic_z_h += diffToDynamic;
  PLOT("module:FLIPMController:dynamic_z_h", dynamic_z_h);

  if (theFLIPMControllerParameter.useRCS)
  {
    return Point(targetCOM.x, targetCOM.y, dynamic_z_h);
  }
  else
  {
    return Point(static_cast<float>(x(X)(3, 0)), static_cast<float>(x(Y)(3, 0)), dynamic_z_h);
  }
}

void FLIPMController::update(TargetCoM& targetCoM)
{
  DECLARE_PLOT("module:FLIPMController:refZMPDiff.x");
  DECLARE_PLOT("module:FLIPMController:reset");
  DECLARE_PLOT("module:FLIPMController:refZMP.x");
  DECLARE_PLOT("module:FLIPMController:refZMP.y");
  DECLARE_PLOT("module:FLIPMController:refZMP_RCS.x");
  DECLARE_PLOT("module:FLIPMController:refZMP_RCS.y");
  DECLARE_PLOT("module:FLIPMController:calcZMP.x");
  DECLARE_PLOT("module:FLIPMController:calcZMP.y");
  DECLARE_PLOT("module:FLIPMController:calcAcc.x");
  DECLARE_PLOT("module:FLIPMController:calcAcc.y");
  DECLARE_PLOT("module:FLIPMController:x1");
  DECLARE_PLOT("module:FLIPMController:x2");
  DECLARE_PLOT("module:FLIPMController:y1");
  DECLARE_PLOT("module:FLIPMController:y2");
  //DECLARE_PLOT("module:FLIPMController:angleX");
  //DECLARE_PLOT("module:FLIPMController:angleY");
  DECLARE_PLOT("module:FLIPMController:targetCoM.x");
  DECLARE_PLOT("module:FLIPMController:targetCoM.y");

  DECLARE_PLOT("module:FLIPMController:calcZMP_RCS.x");
  DECLARE_PLOT("module:FLIPMController:calcZMP_RCS.y");
  DECLARE_PLOT("module:FLIPMController:x1_RCS");
  DECLARE_PLOT("module:FLIPMController:x2_RCS");
  DECLARE_PLOT("module:FLIPMController:y1_RCS");
  DECLARE_PLOT("module:FLIPMController:y2_RCS");
  DECLARE_PLOT("module:FLIPMController:targetCoM_RCS.x");
  DECLARE_PLOT("module:FLIPMController:targetCoM_RCS.y");
  DECLARE_PLOT("module:FLIPMController:targetCoMDiff.x");
  DECLARE_PLOT("module:FLIPMController:targetCoMDiff.y");
  DECLARE_PLOT("module:FLIPMController:dynamic_z_h");

  DECLARE_PLOT("module:FLIPMController:footStepPhase");

  if (!isRunning && theRefZMP2018.running) /* if the controller isn't running, but we have several new reference ZMP's*/
  {
    Start();
    isRunning = true;
  }

  targetCoM.x = targetCoM.y = 0; /* reset the target COM*/
  for (int i = 0; i < 6; i++)
  {
    targetCoM.state_x[i] = 0.0;
    targetCoM.state_y[i] = 0.0;
  }

  if (isRunning && !theRefZMP2018.running) /* if the controller is running, but we have no new reference ZMP's*/
  {
    End();
    isRunning = false;
  }

  MARK("FLIPMController", "x1p"); /*x1 Position*/
  MARK("FLIPMController", "x1s"); /*x1 Speed*/
  MARK("FLIPMController", "x1a"); /*x1 Acceleration*/
  MARK("FLIPMController", "x2p"); /*x2 Position*/
  MARK("FLIPMController", "x2s"); /*x2 Speed*/
  MARK("FLIPMController", "x2a"); /*x2 Acceleration*/
  MARK("FLIPMController", "y1p"); /*y1 Position*/
  MARK("FLIPMController", "y1s"); /*y1 Speed*/
  MARK("FLIPMController", "y1a"); /*y1 Acceleration*/
  MARK("FLIPMController", "y2p"); /*y2 Position*/
  MARK("FLIPMController", "y2s"); /*y2 Speed*/
  MARK("FLIPMController", "y2a"); /*y2 Acceleration*/
  MARK("FLIPMController", "calcZMPx");
  MARK("FLIPMController", "calcZMPy");
  MARK("FLIPMController", "pRefX");
  MARK("FLIPMController", "pRefY");

  LOG("FLIPMController", "CoPx", theZMPModel.ZMP_WCS.x());
  LOG("FLIPMController", "CoPy", theZMPModel.ZMP_WCS.y());

  if (isRunning)
  {
    // Elemente, die vom ZMPGenerator modifiziert wurden und im selben Frame rausgeschickt wurden,
    // werden hier eingehangen und dann nochmal modififiert.
    (Point&)targetCoM = controllerStep(); /* Set the targetCoM*/

    for (int i = 0; i < 6; i++)
    {
      targetCoM.state_x[i] = x(X)(i, 0);
      targetCoM.state_y[i] = x(Y)(i, 0);
    }

    ASSERT(x[0] == x[0]);

    PLOT("module:FLIPMController:x1", x(X)(0, 0)); /*x1 Position*/
    PLOT("module:FLIPMController:x2", x(X)(3, 0)); /*x2 Position*/
    PLOT("module:FLIPMController:y1", x(Y)(0, 0)); /*y1 Position*/
    PLOT("module:FLIPMController:y2", x(Y)(3, 0)); /*y2 Position*/
    PLOT("module:FLIPMController:calcZMP.x", theFLIPMControllerParameter.controllerParamsX.c * x(X));
    PLOT("module:FLIPMController:calcZMP.y", theFLIPMControllerParameter.controllerParamsY.c * x(Y));

    PLOT("module:FLIPMController:x1_RCS", x_RCS(X)(0, 0)); /*x1 Position*/
    PLOT("module:FLIPMController:x2_RCS", x_RCS(X)(3, 0)); /*x2 Position*/
    PLOT("module:FLIPMController:y1_RCS", x_RCS(Y)(0, 0)); /*y1 Position*/
    PLOT("module:FLIPMController:y2_RCS", x_RCS(Y)(3, 0)); /*y2 Position*/
    PLOT("module:FLIPMController:calcZMP_RCS.x", theFLIPMControllerParameter.controllerParamsX.c * x_RCS(X));
    PLOT("module:FLIPMController:calcZMP_RCS.y", theFLIPMControllerParameter.controllerParamsY.c * x_RCS(Y));

    PLOT("module:FLIPMController:calcAcc.y", x(Y)(2, 0));
    PLOT("module:FLIPMController:calcAcc.x", x(X)(2, 0));

    PLOT("module:FLIPMController:footStepPhase",
        (theFootpositions.phase == DWE::WalkingPhase::firstSingleSupport) ? 0.07f : ((theFootpositions.phase == DWE::WalkingPhase::secondSingleSupport) ? -0.07f : 0.f));

    LOG("FLIPMController", "x1p", x(X)(0, 0)); /*x1 Position*/
    LOG("FLIPMController", "x1s", x(X)(1, 0)); /*x1 Speed*/
    LOG("FLIPMController", "x1a", x(X)(2, 0)); /*x1 Acceleration*/
    LOG("FLIPMController", "x2p", x(X)(3, 0)); /*x2 Position*/
    LOG("FLIPMController", "x2s", x(X)(4, 0)); /*x2 Speed*/
    LOG("FLIPMController", "x2a", x(X)(5, 0)); /*x2 Acceleration*/
    LOG("FLIPMController", "y1p", x(Y)(0, 0)); /*y1 Position*/
    LOG("FLIPMController", "y1s", x(Y)(1, 0)); /*y1 Speed*/
    LOG("FLIPMController", "y1a", x(Y)(2, 0)); /*y1 Acceleration*/
    LOG("FLIPMController", "y2p", x(Y)(3, 0)); /*y2 Position*/
    LOG("FLIPMController", "y2s", x(Y)(4, 0)); /*y2 Speed*/
    LOG("FLIPMController", "y2a", x(Y)(5, 0)); /*y2 Acceleration*/
    LOG("FLIPMController", "calcZMPx", theFLIPMControllerParameter.controllerParamsX.c * x(X));
    LOG("FLIPMController", "calcZMPy", theFLIPMControllerParameter.controllerParamsY.c * x(Y));
    LOG("FLIPMController", "pRefX", theRefZMP2018.zmpRCS[0].x());
    LOG("FLIPMController", "pRefY", theRefZMP2018.zmpRCS[0].y());
  }

  targetCoM.isRunning = isRunning;
}

MAKE_MODULE(FLIPMController, dortmundWalkingEngine)
