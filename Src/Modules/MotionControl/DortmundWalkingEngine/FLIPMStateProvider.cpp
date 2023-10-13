#include "FLIPMStateProvider.h"
//#include "CSTransform.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

using namespace DWE;

FLIPMStateProvider::FLIPMStateProvider()
{
  targetCoM1WCS = Vector2f::Zero();
  targetVel1WCS = Vector2f::Zero();
  targetAcc1WCS = Vector2f::Zero();
  targetCoM2WCS = Vector2f::Zero();
  targetVel2WCS = Vector2f::Zero();
  targetAcc2WCS = Vector2f::Zero();

  filteredCoM1 = Vector2f::Zero();
  filteredVel1 = Vector2f::Zero();
  filteredAcc1 = Vector2f::Zero();
  filteredCoM2 = Vector2f::Zero();
  filteredVel2 = Vector2f::Zero();
  filteredAcc2 = Vector2f::Zero();

  targetCoM1DelayBuffer.clear();
  targetVel1DelayBuffer.clear();
  targetAcc1DelayBuffer.clear();
  targetCoM2DelayBuffer.clear();
  targetVel2DelayBuffer.clear();
  targetAcc2DelayBuffer.clear();

  coM1DelayBuffer.clear();
  vel1DelayBuffer.clear();
  acc1DelayBuffer.clear();
  coM2DelayBuffer.clear();
  vel2DelayBuffer.clear();
  acc2DelayBuffer.clear();

  CoM_MRE = Vector3f::Zero();
  CoM_IMU = Vector3f::Zero();
  Acc_IMU = Vector3f::Zero();
  Acc_ZMP = Vector3f::Zero();
}

void FLIPMStateProvider::update(ObservedFLIPMError& observedFLIPMError)
{
  declarePlots();
  updateSensorData();

  /* retrieve robotPosition */
  Pose2f robotPosition(theWalkingInfo.robotPosition.rotation, Vector2f(theWalkingInfo.robotPosition.translation.x(), theWalkingInfo.robotPosition.translation.y()));

  /* retrieve target values (WCS) */
  targetCoM1WCS.x() = static_cast<float>(theTargetCoM.state_x[0]);
  targetCoM1WCS.y() = static_cast<float>(theTargetCoM.state_y[0]);

  targetVel1WCS.x() = static_cast<float>(theTargetCoM.state_x[1]);
  targetVel1WCS.y() = static_cast<float>(theTargetCoM.state_y[1]);

  targetAcc1WCS.x() = static_cast<float>(theTargetCoM.state_x[2]);
  targetAcc1WCS.y() = static_cast<float>(theTargetCoM.state_y[2]);

  targetCoM2WCS.x() = static_cast<float>(theTargetCoM.state_x[3]);
  targetCoM2WCS.y() = static_cast<float>(theTargetCoM.state_y[3]);

  targetVel2WCS.x() = static_cast<float>(theTargetCoM.state_x[4]);
  targetVel2WCS.y() = static_cast<float>(theTargetCoM.state_y[4]);

  targetAcc2WCS.x() = static_cast<float>(theTargetCoM.state_x[5]);
  targetAcc2WCS.y() = static_cast<float>(theTargetCoM.state_y[5]);

  /* rotate target values from WCS in RCS */
  if (useRCS)
  {
    targetCoM1WCS.rotate(-robotPosition.rotation);
    targetVel1WCS.rotate(-robotPosition.rotation);
    targetAcc1WCS.rotate(-robotPosition.rotation);
    targetCoM2WCS.rotate(-robotPosition.rotation);
    targetVel2WCS.rotate(-robotPosition.rotation);
    targetAcc2WCS.rotate(-robotPosition.rotation);
  }

  /* add the target values on the delay buffer */
  targetCoM1DelayBuffer.push_front(targetCoM1WCS);
  targetVel1DelayBuffer.push_front(targetVel1WCS);
  targetAcc1DelayBuffer.push_front(targetAcc1WCS);
  targetCoM2DelayBuffer.push_front(targetCoM2WCS);
  targetVel2DelayBuffer.push_front(targetVel2WCS);
  targetAcc2DelayBuffer.push_front(targetAcc2WCS);

  /* update the state values */
  coM1DelayBuffer.push_front(updateCoM1(robotPosition));
  vel1DelayBuffer.push_front(updateVel1(robotPosition));
  acc1DelayBuffer.push_front(updateAcc1(robotPosition));
  coM2DelayBuffer.push_front(updateCoM2(robotPosition));
  vel2DelayBuffer.push_front(updateVel2(robotPosition));
  acc2DelayBuffer.push_front(updateAcc2(robotPosition));

  /* update the observedFLIPMError for X */
  Vector6f xSensor(coM1DelayBuffer[-1 * std::min(0, delays[0])].x(),
      vel1DelayBuffer[-1 * std::min(0, delays[1])].x(),
      acc1DelayBuffer[-1 * std::min(0, delays[2])].x(),
      coM2DelayBuffer[-1 * std::min(0, delays[3])].x(),
      vel2DelayBuffer[-1 * std::min(0, delays[4])].x(),
      acc2DelayBuffer[-1 * std::min(0, delays[5])].x());

  Vector6f xTarget(targetCoM1DelayBuffer[std::max(0, delays[0])].x(),
      targetVel1DelayBuffer[std::max(0, delays[1])].x(),
      targetAcc1DelayBuffer[std::max(0, delays[2])].x(),
      targetCoM2DelayBuffer[std::max(0, delays[3])].x(),
      targetVel2DelayBuffer[std::max(0, delays[4])].x(),
      targetAcc2DelayBuffer[std::max(0, delays[5])].x());

  observedFLIPMError.observedError.x()[0] = (xSensor[0] - xTarget[0]);
  observedFLIPMError.observedError.x()[1] = (xSensor[1] - xTarget[1]);
  observedFLIPMError.observedError.x()[2] = (xSensor[2] - xTarget[2]);
  observedFLIPMError.observedError.x()[3] = (xSensor[3] - xTarget[3]);
  observedFLIPMError.observedError.x()[4] = (xSensor[4] - xTarget[4]);
  observedFLIPMError.observedError.x()[5] = (xSensor[5] - xTarget[5]);

  PLOT("module:FLIPMStateProvider:CoM1.x", xSensor[0]);
  PLOT("module:FLIPMStateProvider:targetCoM1.x", xTarget[0]);
  PLOT("module:FLIPMStateProvider:CoM1Diff.x", observedFLIPMError.observedError.x()[0]);
  PLOT("module:FLIPMStateProvider:Vel1.x", xSensor[1]);
  PLOT("module:FLIPMStateProvider:targetVel1.x", xTarget[1]);
  PLOT("module:FLIPMStateProvider:Vel1Diff.x", observedFLIPMError.observedError.x()[1]);
  PLOT("module:FLIPMStateProvider:Acc1.x", xSensor[2]);
  PLOT("module:FLIPMStateProvider:targetAcc1.x", xTarget[2]);
  PLOT("module:FLIPMStateProvider:Acc1Diff.x", observedFLIPMError.observedError.x()[2]);
  PLOT("module:FLIPMStateProvider:CoM2.x", xSensor[3]);
  PLOT("module:FLIPMStateProvider:targetCoM2.x", xTarget[3]);
  PLOT("module:FLIPMStateProvider:CoM2Diff.x", observedFLIPMError.observedError.x()[3]);
  PLOT("module:FLIPMStateProvider:Vel2.x", xSensor[4]);
  PLOT("module:FLIPMStateProvider:targetVel2.x", xTarget[4]);
  PLOT("module:FLIPMStateProvider:Vel2Diff.x", observedFLIPMError.observedError.x()[4]);
  PLOT("module:FLIPMStateProvider:Acc2.x", xSensor[5]);
  PLOT("module:FLIPMStateProvider:targetAcc2.x", xTarget[5]);
  PLOT("module:FLIPMStateProvider:Acc2Diff.x", observedFLIPMError.observedError.x()[5]);

  observedFLIPMError.observedError.x() = observedFLIPMError.observedError.x().cwiseProduct(xParams.gains.cast<double>());
  observedFLIPMError.observedError.x() *= xParams.gainsFactor;

  /* update the observedFLIPMError for Y */
  Vector6f ySensor(coM1DelayBuffer[-1 * std::min(0, delays[0])].y(),
      vel1DelayBuffer[-1 * std::min(0, delays[1])].y(),
      acc1DelayBuffer[-1 * std::min(0, delays[2])].y(),
      coM2DelayBuffer[-1 * std::min(0, delays[3])].y(),
      vel2DelayBuffer[-1 * std::min(0, delays[4])].y(),
      acc2DelayBuffer[-1 * std::min(0, delays[5])].y());

  Vector6f yTarget(targetCoM1DelayBuffer[std::max(0, delays[0])].y(),
      targetVel1DelayBuffer[std::max(0, delays[1])].y(),
      targetAcc1DelayBuffer[std::max(0, delays[2])].y(),
      targetCoM2DelayBuffer[std::max(0, delays[3])].y(),
      targetVel2DelayBuffer[std::max(0, delays[4])].y(),
      targetAcc2DelayBuffer[std::max(0, delays[5])].y());

  observedFLIPMError.observedError.y()[0] = (ySensor[0] - yTarget[0]);
  observedFLIPMError.observedError.y()[1] = (ySensor[1] - yTarget[1]);
  observedFLIPMError.observedError.y()[2] = (ySensor[2] - yTarget[2]);
  observedFLIPMError.observedError.y()[3] = (ySensor[3] - yTarget[3]);
  observedFLIPMError.observedError.y()[4] = (ySensor[4] - yTarget[4]);
  observedFLIPMError.observedError.y()[5] = (ySensor[5] - yTarget[5]);

  PLOT("module:FLIPMStateProvider:CoM1.y", ySensor[0]);
  PLOT("module:FLIPMStateProvider:targetCoM1.y", yTarget[0]);
  PLOT("module:FLIPMStateProvider:CoM1Diff.y", observedFLIPMError.observedError.y()[0]);
  PLOT("module:FLIPMStateProvider:Vel1.y", ySensor[1]);
  PLOT("module:FLIPMStateProvider:targetVel1.y", yTarget[1]);
  PLOT("module:FLIPMStateProvider:Vel1Diff.y", observedFLIPMError.observedError.y()[1]);
  PLOT("module:FLIPMStateProvider:Acc1.y", ySensor[2]);
  PLOT("module:FLIPMStateProvider:targetAcc1.y", yTarget[2]);
  PLOT("module:FLIPMStateProvider:Acc1Diff.y", observedFLIPMError.observedError.y()[2]);
  PLOT("module:FLIPMStateProvider:CoM2.y", ySensor[3]);
  PLOT("module:FLIPMStateProvider:targetCoM2.y", yTarget[3]);
  PLOT("module:FLIPMStateProvider:CoM2Diff.y", observedFLIPMError.observedError.y()[3]);
  PLOT("module:FLIPMStateProvider:Vel2.y", ySensor[4]);
  PLOT("module:FLIPMStateProvider:targetVel2.y", yTarget[4]);
  PLOT("module:FLIPMStateProvider:Vel2Diff.y", observedFLIPMError.observedError.y()[4]);
  PLOT("module:FLIPMStateProvider:Acc2.y", ySensor[5]);
  PLOT("module:FLIPMStateProvider:targetAcc2.y", yTarget[5]);
  PLOT("module:FLIPMStateProvider:Acc2Diff.y", observedFLIPMError.observedError.y()[5]);

  observedFLIPMError.observedError.y() = observedFLIPMError.observedError.y().cwiseProduct(yParams.gains.cast<double>());
  observedFLIPMError.observedError.y() *= yParams.gainsFactor;
}

void FLIPMStateProvider::declarePlots()
{
  DECLARE_PLOT("module:FLIPMStateProvider:CoM1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetCoM1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM1Diff.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetVel1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel1Diff.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetAcc1.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc1Diff.x");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetCoM2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM2Diff.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetVel2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel2Diff.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:targetAcc2.x");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc2Diff.x");

  DECLARE_PLOT("module:FLIPMStateProvider:CoM1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetCoM1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM1Diff.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetVel1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel1Diff.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetAcc1.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc1Diff.y");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetCoM2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:CoM2Diff.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetVel2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Vel2Diff.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:targetAcc2.y");
  DECLARE_PLOT("module:FLIPMStateProvider:Acc2Diff.y");
}

void FLIPMStateProvider::updateSensorData()
{
  Point coM_MRE_RCS(theRobotModel.centerOfMass.x() / 1000, theRobotModel.centerOfMass.y() / 1000, (theRobotModel.centerOfMass.z()) / 1000, 0);
  Point CoM_MRE_WCS = theWalkingInfo.toWorldCoords(coM_MRE_RCS);
  CoM_MRE.x() = CoM_MRE_WCS.x;
  CoM_MRE.y() = CoM_MRE_WCS.y;
  CoM_MRE.z() = CoM_MRE_WCS.z;

  Point coM_IMU_RCS(coM_MRE_RCS);
  coM_IMU_RCS.rotateAroundX(theJoinedIMUData.imuData[anglesource].angle.x());
  coM_IMU_RCS.rotateAroundY(theJoinedIMUData.imuData[anglesource].angle.y());
  Point CoM_IMU_WCS = theWalkingInfo.toWorldCoords(coM_IMU_RCS);
  CoM_IMU.x() = CoM_IMU_WCS.x;
  CoM_IMU.y() = CoM_IMU_WCS.y;
  CoM_IMU.z() = CoM_IMU_WCS.z;

  Acc_IMU.x() = theJoinedIMUData.imuData[anglesource].acc.x();
  Acc_IMU.y() = theJoinedIMUData.imuData[anglesource].acc.y();
  Acc_IMU.z() = theJoinedIMUData.imuData[anglesource].acc.z();

  Acc_ZMP.x() = theZMPModel.ZMP_RCS.x() * 1000.f;
  Acc_ZMP.y() = theZMPModel.ZMP_RCS.y() * 1000.f;
  Acc_ZMP.z() = theZMPModel.ZMP_RCS.z() * 1000.f;
}

Vector2f FLIPMStateProvider::updateCoM1(Pose2f& robotPosition)
{
  Vector2f coM1; /* CoM1 Provider (WCS) */
  if (providers.CoM1Provider == StateProvider::MRECoM)
  {
    coM1.x() = CoM_MRE.x();
    coM1.y() = CoM_MRE.y();
  }
  else if (providers.CoM1Provider == StateProvider::IMUCoM)
  {
    coM1.x() = CoM_IMU.x();
    coM1.y() = CoM_IMU.y();
  }
  else if (providers.CoM1Provider == StateProvider::targetCoM)
  {
    coM1.x() = targetCoM1DelayBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1].x();
    coM1.y() = targetCoM1DelayBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1].y();
  }

  /* CoM1 Provider (WCS->RCS) */
  coM1 -= robotPosition.translation;
  coM1.rotate(-robotPosition.rotation);

  /* add offsets to CoM1 which are in RCS */
  coM1.x() += xParams.offset[0];
  coM1.y() += yParams.offset[0];

  coM1.x() *= xParams.scale[0];
  coM1.y() *= yParams.scale[0];

  /* CoM1 Provider (RCS->WCS)*/
  if (!useRCS)
  {
    coM1.rotate(robotPosition.rotation);
    coM1 += robotPosition.translation;
  }

  return coM1;
}

Vector2f FLIPMStateProvider::updateVel1(Pose2f& robotPosition)
{
  Vector2f vel1; /* Vel1 Provider (RCS) */
  if (providers.Vel1Provider == StateProvider::integratedCoM)
  {
    if (coM1DelayBuffer.size() >= 2)
    {
      int delay = -1 * std::min(0, delays[1]);
      vel1.x() = (coM1DelayBuffer[delay].x() - coM1DelayBuffer[delay + 1].x()) / theFrameInfo.cycleTime;
      vel1.y() = (coM1DelayBuffer[delay].y() - coM1DelayBuffer[delay + 1].y()) / theFrameInfo.cycleTime;
    }
    else
    {
      vel1 = Vector2f::Zero();
    }
  }

  float filter = vel1Filter;
  filteredVel1.x() = filteredVel1.x() * (1 - filter) + vel1.x() * filter;
  filteredVel1.y() = filteredVel1.y() * (1 - filter) + vel1.y() * filter;

  vel1.x() = filteredVel1.x();
  vel1.y() = filteredVel1.y();

  vel1.x() += xParams.offset[1];
  vel1.y() += yParams.offset[1];

  vel1.x() *= xParams.scale[1];
  vel1.y() *= yParams.scale[1];

  /* Vel1 Provider (RCS->WCS) */
  if (!useRCS)
  {
    vel1.rotate(robotPosition.rotation);
  }


  return vel1;
}

Vector2f FLIPMStateProvider::updateAcc1(Pose2f& robotPosition)
{
  Vector2f acc1; /* Acc1 Provider (RCS)*/
  if (providers.Acc1Provider == StateProvider::ZMPAcc)
  {
    acc1.x() = Acc_ZMP.x();
    acc1.y() = Acc_ZMP.y();
  }
  else if (providers.Acc1Provider == StateProvider::ZMPCoM1Acc)
  {
    acc1.x() = (-1.f * theZMPModel.ZMP_WCS.x() + coM1DelayBuffer[-1 * std::min(0, delays[0])].x()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsX.z_h);
    acc1.y() = (-1.f * theZMPModel.ZMP_WCS.y() + coM1DelayBuffer[-1 * std::min(0, delays[0])].y()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsY.z_h);
  }
  else if (providers.Acc1Provider == StateProvider::ZMPTargetAcc)
  {
    acc1.x() = (-1.f * theZMPModel.ZMP_WCS.x() + targetCoM1DelayBuffer[std::max(0, delays[0])].x()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsX.z_h);
    acc1.y() = (-1.f * theZMPModel.ZMP_WCS.y() + targetCoM1DelayBuffer[std::max(0, delays[0])].y()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsY.z_h);
  }
  else if (providers.Acc1Provider == StateProvider::targetAcc)
  {
    acc1.x() = targetAcc1DelayBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].x();
    acc1.y() = targetAcc1DelayBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].y();
  }
  else if (providers.Acc1Provider == StateProvider::IMUAcc)
  {
    acc1.x() = Acc_IMU.x();
    acc1.y() = Acc_IMU.y();
  }
  else if (providers.Acc1Provider == StateProvider::integratedAcc)
  {
    if (vel1DelayBuffer.size() >= 2)
    {
      acc1.x() = (vel1DelayBuffer[0].x() - vel1DelayBuffer[1].x()) / theFrameInfo.cycleTime;
      acc1.y() = (vel1DelayBuffer[0].y() - vel1DelayBuffer[1].y()) / theFrameInfo.cycleTime;
    }
    else
    {
      acc1 = Vector2f::Zero();
    }
  }

  float filter = acc1Filter;
  filteredAcc1.x() = filteredAcc1.x() * (1 - filter) + acc1.x() * filter;
  filteredAcc1.y() = filteredAcc1.y() * (1 - filter) + acc1.y() * filter;

  acc1.x() = filteredAcc1.x();
  acc1.y() = filteredAcc1.y();

  acc1.x() += xParams.offset[2];
  acc1.y() += yParams.offset[2];

  acc1.x() *= xParams.scale[2];
  acc1.y() *= yParams.scale[2];

  /* Acc1 Provider (RCS->WCS) */
  if (!useRCS)
  {
    acc1.rotate(robotPosition.rotation);
  }

  return acc1;
}

Vector2f FLIPMStateProvider::updateCoM2(Pose2f& robotPosition)
{
  Vector2f coM2; /* CoM2 Provider (WCS) */
  if (providers.CoM2Provider == StateProvider::MRECoM)
  {
    coM2.x() = CoM_MRE.x();
    coM2.y() = CoM_MRE.y();
  }
  else if (providers.CoM2Provider == StateProvider::IMUCoM)
  {
    coM2.x() = CoM_IMU.x();
    coM2.y() = CoM_IMU.y();
  }
  else if (providers.CoM2Provider == StateProvider::targetCoM)
  {
    coM2.x() = targetCoM2DelayBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1].x();
    coM2.y() = targetCoM2DelayBuffer[theWalkingEngineParams.jointSensorDelayFrames - 1].y();
  }
  /* CoM2 Provider (WCS->RCS) */
  coM2.rotate(-robotPosition.rotation);

  /* add offsets to CoM2 which are in RCS */
  coM2.x() += xParams.offset[3];
  coM2.y() += yParams.offset[3];

  coM2.x() *= xParams.scale[3];
  coM2.y() *= yParams.scale[3];

  /* CoM2 Provider (RCS->WCS)*/
  if (!useRCS)
  {
    coM2.rotate(robotPosition.rotation);
  }

  return coM2;
}

Vector2f FLIPMStateProvider::updateVel2(Pose2f& robotPosition)
{
  Vector2f vel2; /* Vel2 Provider (RCS) */
  if (providers.Vel2Provider == StateProvider::integratedCoM)
  {
    if (coM2DelayBuffer.size() >= 2)
    {
      int delay = -1 * std::min(0, delays[4]);
      vel2.x() = (coM2DelayBuffer[delay].x() - coM2DelayBuffer[delay + 1].x()) / theFrameInfo.cycleTime;
      vel2.y() = (coM2DelayBuffer[delay].y() - coM2DelayBuffer[delay + 1].y()) / theFrameInfo.cycleTime;
    }
    else
    {
      vel2 = Vector2f::Zero();
    }
  }

  float filter = vel1Filter;
  filteredVel2.x() = filteredVel2.x() * (1 - filter) + vel2.x() * filter;
  filteredVel2.y() = filteredVel2.y() * (1 - filter) + vel2.y() * filter;

  vel2.x() = filteredVel2.x();
  vel2.y() = filteredVel2.y();

  vel2.x() += xParams.offset[4];
  vel2.y() += yParams.offset[4];

  vel2.x() *= xParams.scale[4];
  vel2.y() *= yParams.scale[4];

  /* Vel1 Provider (RCS->WCS) */
  if (!useRCS)
  {
    vel2.rotate(robotPosition.rotation);
  }

  return vel2;
}

Vector2f FLIPMStateProvider::updateAcc2(Pose2f& robotPosition)
{
  Vector2f acc2; /* Acc2 Provider (RCS) */
  if (providers.Acc2Provider == StateProvider::ZMPAcc)
  {
    acc2.x() = Acc_ZMP.x();
    acc2.y() = Acc_ZMP.y();
  }
  else if (providers.Acc2Provider == StateProvider::ZMPCoM1Acc)
  {
    acc2.x() = (-1.f * theZMPModel.ZMP_WCS.x() + coM2DelayBuffer.front().x()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsX.z_h);
    acc2.y() = (-1.f * theZMPModel.ZMP_WCS.y() + coM2DelayBuffer.front().y()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsY.z_h);
  }
  else if (providers.Acc2Provider == StateProvider::ZMPTargetAcc)
  {
    acc2.x() = (-1.f * theZMPModel.ZMP_WCS.x() + targetCoM1WCS.x()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsX.z_h);
    acc2.y() = (-1.f * theZMPModel.ZMP_WCS.y() + targetCoM1WCS.y()) * (theWalkCalibration.gravity / theFLIPMParameter.paramsY.z_h);
  }
  else if (providers.Acc2Provider == StateProvider::targetAcc)
  {
    acc2.x() = targetAcc2DelayBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].x();
    acc2.y() = targetAcc2DelayBuffer[theWalkingEngineParams.imuSensorDelayFrames - 1].y();
  }
  else if (providers.Acc2Provider == StateProvider::IMUAcc)
  {
    acc2.x() = Acc_IMU.x();
    acc2.y() = Acc_IMU.y();
  }
  else if (providers.Acc2Provider == StateProvider::integratedAcc)
  {
    if (vel2DelayBuffer.size() >= 2)
    {
      acc2.x() = (vel2DelayBuffer[0].x() - vel2DelayBuffer[1].x()) / theFrameInfo.cycleTime;
      acc2.y() = (vel2DelayBuffer[0].y() - vel2DelayBuffer[1].y()) / theFrameInfo.cycleTime;
    }
    else
    {
      acc2 = Vector2f::Zero();
    }
  }

  float filter = acc2Filter;
  filteredAcc2.x() = filteredAcc2.x() * (1 - filter) + acc2.x() * filter;
  filteredAcc2.y() = filteredAcc2.y() * (1 - filter) + acc2.y() * filter;

  acc2.x() = filteredAcc2.x();
  acc2.y() = filteredAcc2.y();

  acc2.x() += xParams.offset[5];
  acc2.y() += yParams.offset[5];

  acc2.x() *= xParams.scale[5];
  acc2.y() *= yParams.scale[5];

  /* Acc2 Provider (RCS->WCS) */
  if (!useRCS)
  {
    acc2.rotate(robotPosition.rotation);
  }

  return acc2;
}

MAKE_MODULE(FLIPMStateProvider, dortmundWalkingEngine)
