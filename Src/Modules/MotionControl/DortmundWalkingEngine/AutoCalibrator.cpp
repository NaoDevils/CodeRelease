#include "AutoCalibrator.h"
#include "Tools/Settings.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Platform/File.h"

AutoCalibrator::AutoCalibrator()
{
  calibrationOK = false;
  timeStampCalibrationOK = 0;
  calibrate = false;
}


AutoCalibrator::~AutoCalibrator()
{

}

void AutoCalibrator::init(WalkCalibration& walkCalibration)
{
  filteredOriErr = Vector3f(0, 0, 0);
  filteredPosErr = Vector3f(0, 0, 0);
  lastPosCorrection = Vector3f(0, 0, 0);
  lastOriCorrection = RotationMatrix();
  timeCalibrationStart = theFrameInfo.time;
  timeStampCalibrationOK = theFrameInfo.time;
  walkCalibration.fixedOffset.x = 0;
  walkCalibration.fixedOffset.y = 0;
  walkCalibration.fixedOffset.z = 0;
  walkCalibration.fixedOffset.rx = 0;
  walkCalibration.fixedOffset.ry = 0;
  walkCalibration.fixedOffset.r = 0;
  walkCalibration.calibrationDone = false;
}

void AutoCalibrator::saveCalibration(WalkCalibration& walkCalibration)
{
  std::list<std::string> names = File::getFullNames("walkCalibration.cfg");
  bool foundFile = false;
  std::string fullPath = "walkCalibration.cfg";
  for (auto& path : names)
  {
    InMapFile file(path.c_str());
    if (file.exists())
    {
      foundFile = true;
      fullPath = path;
      break;
    }
  }
  if (foundFile)
  {
    OutMapFile stream(fullPath);
    stream << walkCalibration;
  }
  else
    OUTPUT_ERROR("AutoCalibrator: WalkCalibration could not be saved!");
}

void AutoCalibrator::loadCalibration(WalkCalibration& walkCalibration)
{
  std::list<std::string> names = File::getFullNames("walkCalibration.cfg");
  bool foundFile = false;
  std::string fullPath = "walkCalibration.cfg";
  for (auto& path : names)
  {
    InMapFile file(path.c_str());
    if (file.exists())
    {
      foundFile = true;
      fullPath = path;
      break;
    }
  }
  if (foundFile)
  {
    InMapFile stream(fullPath);
    stream >> walkCalibration;
  }
  else
  {
    OUTPUT_ERROR("AutoCalibrator: WalkCalibration could not be loaded!");
    walkCalibration = WalkCalibration();
  }
  
}


void AutoCalibrator::update(WalkCalibration& walkCalibration)
{
  DECLARE_PLOT("module:AutoCalibrator:posErr.x");
  DECLARE_PLOT("module:AutoCalibrator:posErr.x");
  DECLARE_PLOT("module:AutoCalibrator:posErr.z");
  DECLARE_PLOT("module:AutoCalibrator:rotErr.x");
  DECLARE_PLOT("module:AutoCalibrator:rotErr.y");
  DECLARE_PLOT("module:AutoCalibrator:rotErr.z");

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:calibrate")
  {
    calibrate = true;
    calibrationOK = false;
    init(walkCalibration);
  }

  DEBUG_RESPONSE_ONCE("module:AutoCalibrator:saveCalibration")
  {
    walkCalibration.calibrationDone = true;
    saveCalibration(walkCalibration);
  }

  if (!calibrationLoaded)
    loadCalibration(walkCalibration);

  if (walkCalibration.calibrationDone || !theWalkingInfo.isRunning || !calibrate)
  {
    timeStampCalibrationOK = theFrameInfo.time;
    return;
  }

  if (theSpeedInfo.speed.translation.x() != 0 || theSpeedInfo.speed.translation.y() != 0 || theSpeedInfo.speed.rotation != 0)
  {
    timeCalibrationStart = theFrameInfo.time;
    timeStampCalibrationOK = theFrameInfo.time;
    return;
  }

  if (theFrameInfo.getTimeSince(timeCalibrationStart) < 5000)
  {
    timeStampCalibrationOK = theFrameInfo.time;
    return;
  }

  // Olli
  int footNum = theWalkingInfo.onFloor[1];
  Vector3f footPos_a = (footNum ? theRobotModel.soleRight : theRobotModel.soleLeft).translation;

  RobotModel desiredModel;
  desiredModel.setJointData(theJointRequest, theRobotDimensions, theMassCalibration);
  Vector3f footPos_d = (footNum ? desiredModel.soleRight : desiredModel.soleLeft).translation;
  // Rotate around CoP
  Vector3f CoPtoCoM_a = -theZMPModel.zmp_acc + theRobotModel.centerOfMass;
  RotationMatrix IMUOrientation = RotationMatrix(0, theInertialSensorData.angle.y(), theInertialSensorData.angle.x());
  Vector3f rotatedCoPtoCoM_a = IMUOrientation * CoPtoCoM_a;
  Vector3f rotatedCoM_a = rotatedCoPtoCoM_a + theZMPModel.zmp_acc;
  Vector3f footToCoM_a = -footPos_a + rotatedCoM_a;
  Vector3f desiredPos = desiredModel.centerOfMass;
  Vector3f footToCoM_d = -footPos_d + desiredPos;
  Vector3f posErr = (footToCoM_a - footToCoM_d) / 1000 - lastPosCorrection;
  Limbs::Limb l = footNum ? Limbs::footRight : Limbs::footLeft;
  //RotationMatrix actualBodyRot = theRobotModel.limbs[l].rotation.inverse(); unused
  RotationMatrix desiredBodyRot = desiredModel.limbs[l].rotation.inverse();
  // IMU shows the complete actual orientation, so use this the determine
  // the overall error.
  RotationMatrix rotErr = lastOriCorrection.inverse() * IMUOrientation.inverse() * desiredBodyRot;
  Vector3f rotErrVec(rotErr.getXAngle(), rotErr.getYAngle(), rotErr.getZAngle());

  // Now filter the stuff
  //filteredPosErr = posErrAlpha * filteredPosErr + (1 - posErrAlpha) * posErr;
  //filteredOriErr = rotErrAlpha * filteredOriErr + (1 - rotErrAlpha) * rotErrVec;

  // The error is now the calibration (*-1)
  walkCalibration.fixedOffset.x = filteredPosErr.x();
  walkCalibration.fixedOffset.y = filteredPosErr.y();
  walkCalibration.fixedOffset.z = filteredPosErr.z();
  walkCalibration.fixedOffset.rx = -filteredOriErr.x();
  walkCalibration.fixedOffset.ry = -filteredOriErr.y();

  //Ingmar
  desiredPositionOffset = Vector3f(theWalkingEngineParams.comOffsets.xFixed*1000, theWalkingEngineParams.comOffsets.yFixed*1000, theRobotModel.centerOfMass.z());
  desiredRotationOffset = Vector3f(0, theWalkingEngineParams.comOffsets.tiltFixed, 0);
  posErr = theRobotModel.centerOfMass - desiredPositionOffset;
  rotErrVec = Vector3f(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0) - desiredRotationOffset;

  // Now filter the stuff
  filteredPosErr.x() = posErrAlpha * filteredPosErr.x() + (1 - posErrAlpha) * posErr.x();
  filteredPosErr.y() = posErrAlpha * filteredPosErr.y() + (1 - posErrAlpha) * posErr.y();
  filteredPosErr.z() = posErrAlpha * filteredPosErr.z() + (1 - posErrAlpha) * posErr.z();
  
  filteredOriErr.x() = rotErrAlpha * filteredOriErr.x() + (1 - rotErrAlpha) * rotErrVec.x();
  filteredOriErr.y() = rotErrAlpha * filteredOriErr.y() + (1 - rotErrAlpha) * rotErrVec.y();

  lastPosCorrection = filteredPosErr;
  lastOriCorrection = rotErr;

  // The error is now the calibration (*-1)
  walkCalibration.fixedOffset.x = filteredPosErr.x()/1000;
  walkCalibration.fixedOffset.y = filteredPosErr.y()/1000;
  walkCalibration.fixedOffset.z = filteredPosErr.z() / 1000;
  walkCalibration.fixedOffset.rx = -filteredOriErr.x();
  walkCalibration.fixedOffset.ry = -filteredOriErr.y();

  if (std::abs(posErr.x()) < maxPosError && std::abs(posErr.y()) < maxPosError && std::abs(posErr.z()) < maxPosError
    && std::abs(rotErrVec.x()) < maxOriError && std::abs(rotErrVec.y()) < maxOriError)
  {
    calibrationOK = true;
  }
  else
    timeStampCalibrationOK = theFrameInfo.time;

  if (theFrameInfo.getTimeSince(timeStampCalibrationOK) > 3000)
  {
    calibrate = false;
    walkCalibration.calibrationDone = true;
    saveCalibration(walkCalibration);
  }
  
  PLOT("module:AutoCalibrator:posErr.x", posErr.x());
  PLOT("module:AutoCalibrator:posErr.x", posErr.y());
  PLOT("module:AutoCalibrator:posErr.z", posErr.z());
  PLOT("module:AutoCalibrator:rotErr.x", rotErrVec.y());
  PLOT("module:AutoCalibrator:rotErr.y", rotErrVec.z());
  PLOT("module:AutoCalibrator:rotErr.z", rotErrVec.x());
}

MAKE_MODULE(AutoCalibrator, dortmundWalkingEngine)
